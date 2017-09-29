#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/pci.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/i2c.h>

#define	RDFPGAJOY_NAME		"rdfpgajoy"

MODULE_AUTHOR("James Covey-Crump <james.covey-crump@spx.com>");
MODULE_DESCRIPTION("I2C Joystick Driver for RD DVC2 FPGA");
MODULE_VERSION("0.1");
MODULE_LICENSE("GPL");

#ifdef GIT_REVISION
MODULE_INFO(gitrev, GIT_REVISION);
#endif

static int suppress_i2c = 0;
module_param(suppress_i2c, int, 0644);
MODULE_PARM_DESC(suppress_i2c, " set to non-zero to suppress I2C traffic");

static uint poll_rate = 10;
module_param(poll_rate, uint, 0644);
MODULE_PARM_DESC(poll_rate, " i2c poll rate (per second)");

static int joy_min = 0x000;
module_param(joy_min, int, 0444);
MODULE_PARM_DESC(joy_min, " joystick minimum value (overridden by dt)");

static int joy_max = 0xFFF;
module_param(joy_max, int, 0444);
MODULE_PARM_DESC(joy_max, " joystick maximum value (overridden by dt)");

static int joy_fuzz = 0x020;
module_param(joy_fuzz, int, 0444);
MODULE_PARM_DESC(joy_fuzz, " joystick fuzz (overridden by dt)");

static struct workqueue_struct *wq;

struct rdfpgajoy_data {
	struct input_dev *input_dev;
	struct i2c_client *i2c_client;
	struct delayed_work dwork;
	bool is_rightjoy;
	bool calibrate;
	int center_x;
	int center_y;
	int min;
	int max;
	int fuzz;
	int i2c_failures;
};

static void rdfpgajoy_input_caps(struct input_dev *input, bool is_rightjoy,
				 int min, int max, int fuzz)
{
	/* we provide key (button) events, absolute position events */
	set_bit(EV_KEY, input->evbit);
	set_bit(EV_ABS, input->evbit);

	if (is_rightjoy) {
		/* clear opposing side (left) event masks */
		clear_bit(BTN_THUMBL, input->keybit);
		clear_bit(ABS_X, input->absbit);
		clear_bit(ABS_Y, input->absbit);

		/* Specify what key events will be raised */
		set_bit(BTN_THUMBR, input->keybit);

		/* Specify axis numeric range - axis,min,max,fuzz,flat */
		input_set_abs_params(input, ABS_RX, min, max, fuzz, 0);
		input_set_abs_params(input, ABS_RY, min, max, fuzz, 0);
	} else {
		/* clear opposing side (right) event masks */
		clear_bit(BTN_THUMBR, input->keybit);
		clear_bit(ABS_RX, input->absbit);
		clear_bit(ABS_RY, input->absbit);

		/* Specify what key events will be raised */
		set_bit(BTN_THUMBL, input->keybit);

		/* Specify numeric range -   axis,  min, max,fuzz,flat */
		input_set_abs_params(input, ABS_X, min, max, fuzz, 0);
		input_set_abs_params(input, ABS_Y, min, max, fuzz, 0);
	}
}

static void raise_events(struct rdfpgajoy_data *p, int x, int y, int btn)
{
	if (p->is_rightjoy) {
		input_report_abs(p->input_dev, ABS_RX, x);
		input_report_abs(p->input_dev, ABS_RY, y);
		input_report_key(p->input_dev, BTN_THUMBR, btn);
		input_sync(p->input_dev);
	} else {
		input_report_abs(p->input_dev, ABS_X, x);
		input_report_abs(p->input_dev, ABS_Y, y);
		input_report_key(p->input_dev, BTN_THUMBL, btn);
		input_sync(p->input_dev);
	}
}

static ssize_t joyside_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buffer, size_t count)
{
	struct rdfpgajoy_data *p = dev_get_drvdata(dev);

	if (buffer[0] != 'l' && buffer[0] != 'r')
		return -EINVAL;

	p->is_rightjoy = (buffer[0] == 'r');
	rdfpgajoy_input_caps(p->input_dev, p->is_rightjoy,
			     p->min, p->max, p->fuzz);

	return count;
}

static ssize_t joyside_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buffer)
{
	char c = 'l';
	struct rdfpgajoy_data *p = dev_get_drvdata(dev);

	if (p->is_rightjoy)
		c = 'r';

	return scnprintf(buffer, PAGE_SIZE, "%c\n", c);
}

static ssize_t inject_event_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buffer, size_t count)
{
	struct rdfpgajoy_data *p = dev_get_drvdata(dev);
	int x, y, btn, num_ret;

	num_ret = sscanf(buffer, "%d%d%d", &x, &y, &btn);
	if (num_ret == 3) {
		raise_events(p, x, y, btn);
		dev_info(dev, "Event Raised %d %d %d\n", x, y, btn);
	} else {
		dev_warn(dev, "Bad format for event injection - 3 ints must be supplied\n");
	}

	return count;
}

/* Attach the sysfs write method */
DEVICE_ATTR_WO(inject_event);	/* inject_event_store() */
DEVICE_ATTR_RW(joyside);	/* joyside_store(), joyside_show() */

/* Attribute Descriptor */
static struct attribute *rdfpgajoy_sysfs_attrs[] = {
	&dev_attr_inject_event.attr,
	&dev_attr_joyside.attr,
	NULL
};

/* Attribute group */
static struct attribute_group rdfpgajoy_attr_group = {
	.attrs = rdfpgajoy_sysfs_attrs,
};


/**
 * normalise_axis - scale incoming value so that the calibrated center point
 *                  is halfway between min and max
 * @v: value to scale
 * @min: minimum
 * @c: calibrated center point
 * @max: maximum
 *
 * Returns: modified value 
 *
 * Joysticks can rest slightly off center.  Calibration occurs at startup to
 * obtain this value.  Any readings are linearly adjusted so that readings at
 * the calibration center is halfway between min and max (ideal_c below)
 * whilst preserving min and max.
 */
static int normalise_axis(int v, int min, int c, int max)
{
	int ideal_c = (min+max)/2;

	/* protect against divide by zero */
	if (c == max || c == min )
		return c;

	/* limit v to min and max */
	v = (v > max ? max : v);
	v = (v < min ? min : v);
	
	if (v > c)
		v = (v-c)*ideal_c/(max-c) + ideal_c;
	else
		v = v*(ideal_c-min)/(c-min);

	return v;
}

static bool rdfpgajoy_i2cread(struct rdfpgajoy_data *p, int *x, int *y, int *btn)
{
	struct device *dev = &p->i2c_client->dev;

	if (suppress_i2c) {
		pr_debug("%s: suppressed i2c read to 0x%02x\n", __func__, 
		         p->i2c_client->addr);
	} else {
		char buf[4];

		if (i2c_master_recv(p->i2c_client, buf, sizeof(buf)) != 4) {
			p->i2c_failures++;
		} else {
			/* y coord first, x coord second on i2c message */
			*y = (((int)buf[0] << 8) | (int)buf[1]) & 0xFFF;
			*x = (((int)buf[2] << 8) | (int)buf[3]) & 0xFFF;
			
			if (btn)
				*btn = 0;
			
			p->i2c_failures = 0;
		}
	}

	/* limit the number of error messages */
	if (p->i2c_failures>1 && p->i2c_failures<=5) {
		dev_warn(dev, "%s: i2c recv failed for address 0x%02x\n",
			 __func__, p->i2c_client->addr);
	}

	return (p->i2c_failures == 0);
}

static void rdfpgajoy_work(struct work_struct *work)
{
	/* suppled work struct is contained within struct rdfpgajoy_data */
	struct delayed_work *dwork = to_delayed_work(work);
	struct rdfpgajoy_data *p = container_of(dwork,
						struct rdfpgajoy_data,
						dwork);
	struct device *dev = &p->i2c_client->dev;

	int x=0, y=0, btn;

	if (rdfpgajoy_i2cread(p, &x, &y, &btn)) {
		if (p->calibrate) {
			p->center_x = x;
			p->center_y = y;
			p->calibrate = false;
			dev_info(dev, "calibrated %s joystick at 0x%02x to mid point 0x%x,0x%x\n", 
			         (p->is_rightjoy ? "right" : "left"),
			         p->i2c_client->addr,
			         x, y);
		}

		x = normalise_axis(x, p->min, p->center_x, p->max);
		y = normalise_axis(y, p->min, p->center_y, p->max);

		raise_events(p, x, y, btn);
	}

	if (p->i2c_failures < 5) {
		/* queue same work item to run again */
		queue_delayed_work(wq, dwork,
				   HZ / (unsigned long)poll_rate);
	} else {
		/* in the case of too many failures...
		 * queue same work item to run again in 10 seconds time
		 * (to avoid flooding the log with attempts)
		 */
		queue_delayed_work(wq, dwork, HZ * 10UL);
	}
}

/* Driver Initialisation */
static int rdfpgajoy_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct rdfpgajoy_data *p;
	struct device *dev = &client->dev;
	struct input_dev *input;
	struct device_node *np = dev->of_node;

	int error;

	p = devm_kzalloc(dev, sizeof(struct rdfpgajoy_data), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	/* Create a sysfs node to read simulated coordinates */
	error = sysfs_create_group(&dev->kobj, &rdfpgajoy_attr_group);
	if (error) {
		dev_err(dev, "Unable create sysfs entry\n");
		return error;
	}

	/* Allocate an input device data structure */
	input = input_allocate_device();
	if (!input) {
		dev_err(dev, "Bad input_alloc_device()\n");
		sysfs_remove_group(&dev->kobj, &rdfpgajoy_attr_group);
		return -ENOMEM;		/* safe as devm_kzalloc used above */
	}

	input->id.bustype = BUS_I2C;
	input->id.vendor  = 0x0001;
	input->id.product = 0x001;
	input->id.version = 0x0001;
	input->name = RDFPGAJOY_NAME;

	p->calibrate = false;
	p->min = joy_min;
	p->max = joy_max;
	p->fuzz = joy_fuzz;

	if (np) {
		u32 v;

		p->calibrate = of_property_read_bool(np, "calibrate");
		p->is_rightjoy = of_property_read_bool(np, "joystick,right");

		if (of_property_read_u32(np, "poll_rate", &v) == 0)
			poll_rate = (uint)v;

		if (of_property_read_u32(np, "joy_min", &v) == 0)
			p->min = (int)v;

		if (of_property_read_u32(np, "joy_max", &v) == 0)
			p->max = (int)v;

		if (of_property_read_u32(np, "joy_fuzz", &v) == 0)
			p->fuzz = (int)v;

	}

	/* set center values, overwritten by calibration if selected */
	p->center_x = (p->min+p->max)/2;
	p->center_y = p->center_x;

	p->i2c_client = client;
	p->input_dev = input;

	/* Set capabilities KEYS, ABS values */
	rdfpgajoy_input_caps(p->input_dev, p->is_rightjoy,
			     p->min, p->max, p->fuzz);

	/* Register with the input subsystem */
	error = input_register_device(input);
	if (error) {
		dev_err(dev, "Unable to input_register_device()\n");
		sysfs_remove_group(&dev->kobj, &rdfpgajoy_attr_group);
		return -ENOMEM;		/* safe as devm_kzalloc used above */
	}

	input_set_drvdata(input, p);
	i2c_set_clientdata(client, p);

	INIT_DELAYED_WORK(&p->dwork, rdfpgajoy_work);

	/* start repeated calls specified by p->dwork */
	queue_delayed_work(wq, &p->dwork,
			   HZ / (unsigned long)poll_rate);

	dev_info(dev, "RD FPGA Joystick Driver Initialised\n");
	return 0;
}

/* Driver Exit */
static int rdfpgajoy_remove(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct rdfpgajoy_data *p = i2c_get_clientdata(client);

	cancel_delayed_work(&p->dwork);

	/* Unregister from the input subsystem */
	input_unregister_device(p->input_dev);

	/* Cleanup sysfs node */
	sysfs_remove_group(&dev->kobj, &rdfpgajoy_attr_group);

	dev_info(dev, "RD FPGA Joystick Driver Removed\n");
	return 0;
}


static const struct i2c_device_id rdfpgajoy_id[] = {
	{ RDFPGAJOY_NAME, 0001, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rdfpgajoy_id);

static const struct of_device_id rdfpgajoy_dt_ids[] = {
	{ .compatible = "rd,rdfpgajoy", },
	{ }
};
MODULE_DEVICE_TABLE(of, rdfpgajoy_dt_ids);

static struct i2c_driver rdfpgajoy_driver = {
	.driver = {
		.name	= RDFPGAJOY_NAME,
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(rdfpgajoy_dt_ids),
	},
	.probe		= rdfpgajoy_probe,
	.remove         = rdfpgajoy_remove,
	.id_table	= rdfpgajoy_id,
};

static int __init rdfpgajoy_init(void)
{
	wq = create_singlethread_workqueue(RDFPGAJOY_NAME "_wq");
	if (!wq)
		return -ENOMEM;

	return i2c_add_driver(&rdfpgajoy_driver);
}
subsys_initcall(rdfpgajoy_init);

static void __exit rdfpgajoy_exit(void)
{
	i2c_del_driver(&rdfpgajoy_driver);
	flush_workqueue(wq);
	destroy_workqueue(wq);
}
module_exit(rdfpgajoy_exit);
