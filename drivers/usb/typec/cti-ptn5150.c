/*
 * cti-ptn5150.c - NXP PTN5150 USB-C Controller Driver
 *
 * Copyright (c) 2022-2023, Connect Tech Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

//#define DEBUG
//Uncomment to print i2c reads/writes to dmesg
//#define DEBUG_READS_WRITES


#include <linux/types.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>

#include <linux/regulator/driver.h>
#include <linux/regulator/fixed.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/machine.h>

#include <linux/usb/role.h>
#include <linux/usb/typec.h>


//Registers
#define PTN5150_ID_REG          0x01
#define PTN5150_CTRL_REG        0x02
#define PTN5150_CABLE_INT_REG   0x03
#define PTN5150_CC_STATUS_REG   0x04
#define PTN5150_CON_DET_REG     0x09
#define PTN5150_VCONN_STAT_REG  0x0A
#define PTN5150_RESET_REG       0x10
#define PTN5150_INT_MASK_REG    0x18
#define PTN5150_INT_STAT_REG    0x19

//Register Bits
#define PTN5150_INT_ATTACH      BIT(0)
#define PTN5150_INT_DETACH      BIT(1)

#define PTN5150_CC_VBUS_DETECT  BIT(7)


enum ptn5150_cc_status
{
    PTN5150_CC_NOT_CONN,
    PTN5150_CC_DFP,
    PTN5150_CC_UFP,
    PTN5150_CC_AUDIO,
    PTN5150_CC_DEBUG,
    PTN5150_CC_INVALID,
};

struct ptn5150
{
    uint8_t addr;   //I2C address
    struct i2c_client*  client;
    struct regmap*      regmap;
    struct device*      dev;

    //When optional is true, no error if not found
    bool optional;

    enum typec_pwr_opmode   pwr_opmode;
    struct typec_capability caps;

    enum ptn5150_cc_status curr_status;

    struct typec_port*      port;
    struct typec_partner*   partner;
    struct usb_role_switch* role_switch;
};


static const struct regmap_config ptn5150_regmap_config = {
    .reg_bits = 8,
    .val_bits = 8,
};

static const char* opmode_to_str(enum typec_pwr_opmode opmode)
{
    switch(opmode){
        case TYPEC_PWR_MODE_USB:
        return "900mA";
        case TYPEC_PWR_MODE_1_5A:
        return "1.5A";
        case TYPEC_PWR_MODE_3_0A:
        return "3.0A";
        case TYPEC_PWR_MODE_PD:
        return "Power Delivery";
    }

    return "Unknown";
}

static const char* port_type_to_str(enum typec_port_data type)
{
    switch(type){
        case TYPEC_PORT_UFP:
        return "Device/UFP";
        case TYPEC_PORT_DFP:
        return "Host/DFP";
        case TYPEC_PORT_DRD:
        return "OTG/DRP";
    }

    return "Unknown";
}

//Convience function for getting the irq from the i2c_client
static inline int ptn5150_get_irq(struct ptn5150* ptn5150)
{
    if(!ptn5150 || !ptn5150->client){
        return -1;
    }

    return ptn5150->client->irq;
}

//Return true if port is configured as host only 
static inline bool ptn5150_is_host_only(struct ptn5150* ptn5150)
{
    if(!ptn5150){
        return true;
    }

    return (ptn5150->caps.data == TYPEC_PORT_DFP);
}

static int ptn5150_write_reg(struct ptn5150* ptn5150, uint8_t reg, uint8_t val)
{
    if(!ptn5150 || !ptn5150->regmap){
        return -EINVAL;
    }

#ifdef DEBUG_READS_WRITES
    dev_info(ptn5150->dev, "Write REG: 0x%02X, VAL 0x%02X\n", reg, val);
#endif

    return regmap_write(ptn5150->regmap, reg, (uint32_t)val);
}

static int ptn5150_read_reg(struct ptn5150* ptn5150, uint8_t reg, uint8_t* val)
{
    int ret;
    uint32_t reg_value;

    if(!ptn5150 || !ptn5150->regmap || !val){
        return -EINVAL;
    }

    ret = regmap_read(ptn5150->regmap, reg, &reg_value);

    if(ret != 0){
        return ret;
    }

    *val = reg_value & 0xFF;

#ifdef DEBUG_READS_WRITES
    dev_info(ptn5150->dev, "Read REG: 0x%02X, VAL 0x%02X\n", reg, *val);
#endif

    return 0;
}


//Attempt to read the ID register to see if device is on the i2c bus
static bool ptn5150_detect(struct ptn5150* ptn5150)
{
    uint8_t value;

    if(!ptn5150){
        return false;
    }

    return (ptn5150_read_reg(ptn5150, PTN5150_ID_REG, &value) == 0);
}

//Write the advertised current limit based on the power op mode
int ptn5150_write_cur_limit(struct ptn5150* ptn5150,
                            enum typec_pwr_opmode limit)
{
    uint8_t value;   //register value

    if(!ptn5150){
        return -EINVAL;
    }
    
    if(ptn5150_read_reg(ptn5150, PTN5150_CTRL_REG, &value) != 0){
        return -EIO;
    } 

    /*
        bits 3 and 4 are used for current limit. Clear them to 
        a known state before setting the desired bits
    */
    value &= ~GENMASK(4, 3);    

    switch(limit){
        case TYPEC_PWR_MODE_USB:   //00
        break;   
        case TYPEC_PWR_MODE_1_5A:  //01
            value |= BIT(3);
        break;
        case TYPEC_PWR_MODE_3_0A:  //10
            value |= BIT(4);
        break;
        default:
            dev_err(ptn5150->dev, "Invalid Current Limit %d\n", limit);
        return -EINVAL;
    }

    dev_dbg(ptn5150->dev, "Setting Advertised Current Limit to %s\n",
            opmode_to_str(limit));

    if(ptn5150_write_reg(ptn5150, PTN5150_CTRL_REG, value) != 0){
        return -EIO;
    } 

    return 0;
}


//Read the Advertised Current Limit from the device
int ptn5150_read_cur_limit(struct ptn5150* ptn5150,
                           enum typec_pwr_opmode* limit)
{
    uint8_t value;   //register value

    if(!ptn5150 || !limit){
        return -EINVAL;
    }
    
    if(ptn5150_read_reg(ptn5150, PTN5150_CTRL_REG, &value) != 0){
        return -EIO;
    } 

    //bits 3 and 4 are used for current limit.
    value &= GENMASK(4, 3);    
    value >>= 3;

    switch(value){
        case 0: //00
            *limit = TYPEC_PWR_MODE_USB;
        break;   
        case 1: //01
            *limit = TYPEC_PWR_MODE_1_5A;
        break;
        case 2: //10
            *limit = TYPEC_PWR_MODE_3_0A;
        break;
        default:
        return -EFAULT;
    }

    return 0;
}

/*
 * Set the controllers port mode via I2C. This will override 
 * the power on default set by the PTN5150 PORT pin 
 */
static int ptn5150_write_port_mode(struct ptn5150* ptn5150,
                                   enum typec_port_data mode)
{
    uint8_t value;   //register value

    if(!ptn5150){
        return -EINVAL;
    }
    
    if(ptn5150_read_reg(ptn5150, PTN5150_CTRL_REG, &value) != 0){
        return -EIO;
    } 

    //bits 1 and 2 are mode, clear first
    value &= ~GENMASK(2, 1);    

    switch(mode){
        case TYPEC_PORT_UFP:    //00
        break;
        case TYPEC_PORT_DFP:    //01
            value |= BIT(1);
        break;
        case TYPEC_PORT_DRD:    //10
            value |= BIT(2);
        break;
    }

    dev_dbg(ptn5150->dev, "Setting Mode to %s\n", port_type_to_str(mode));

    if(ptn5150_write_reg(ptn5150, PTN5150_CTRL_REG, value) != 0){
        return -EIO;
    } 

    return 0;
}

static int ptn5150_get_cc_status(struct ptn5150* ptn5150, 
                                 enum ptn5150_cc_status* status)
{
    uint8_t value;    

    if(!ptn5150 || !status){
        return -EINVAL;
    }

    //read the CC Status register to get cable status
    if(ptn5150_read_reg(ptn5150, PTN5150_CC_STATUS_REG, &value) != 0){
        dev_err(ptn5150->dev, "Failed to read Cable Status from device\n");
        return -EIO;
    }

    //Port Attachment Status is bits 2->4
    value &= GENMASK(4,2);
    value >>= 2; 

   *status = value;

    return 0;
}


static bool ptn5150_cable_detected(struct ptn5150* ptn5150)
{
    enum ptn5150_cc_status cc;

    if(!ptn5150){
        return false;
    }


    if(ptn5150_get_cc_status(ptn5150, &cc) != 0){
        return false;
    }
    
    return (cc == PTN5150_CC_DFP || cc == PTN5150_CC_UFP);
}

int ptn5150_attach(struct ptn5150* ptn5150)
{
    struct typec_partner_desc desc = { };   //zero out desc, we do not set anything
    enum ptn5150_cc_status cc_status;
    enum typec_data_role data_role;
    enum usb_role usb_role;
    enum typec_role power_role;
    int ret;
    if(!ptn5150){
        return -EINVAL;
    }

    if(ptn5150_get_cc_status(ptn5150, &cc_status) != 0){
        dev_err(ptn5150->dev, "Error reading CC Status\n");
        return -EIO;
    }

    if(cc_status == ptn5150->curr_status){
        dev_dbg(ptn5150->dev, "Status same as current status\n");
        return 0;
    }

    //update status
    ptn5150->curr_status = cc_status;

    switch(cc_status)
    {
        case PTN5150_CC_NOT_CONN:
            dev_dbg(ptn5150->dev, "Not Connected\n");
            usb_role = USB_ROLE_NONE;
            data_role = TYPEC_HOST;
            power_role = TYPEC_SOURCE;
        break;
        case PTN5150_CC_DFP:
            dev_dbg(ptn5150->dev, "DFP/Host Connected\n");
            usb_role = USB_ROLE_DEVICE;
            data_role = TYPEC_DEVICE;
            power_role = TYPEC_SINK;
        break;
        case PTN5150_CC_UFP:
            dev_dbg(ptn5150->dev, "UFP/Device Connected\n");
            usb_role = USB_ROLE_HOST;
            data_role = TYPEC_HOST;
            power_role = TYPEC_SOURCE;
        break;
        default:
            dev_info(ptn5150->dev, "Unsupported Device Connected\n");
        return -ENODEV;
    }

	if(!IS_ERR_OR_NULL(ptn5150->port)){
		typec_set_pwr_role(ptn5150->port, power_role);
		typec_set_data_role(ptn5150->port, data_role);
	}

    if(!IS_ERR_OR_NULL(ptn5150->role_switch)){
       ret = usb_role_switch_set_role(ptn5150->role_switch, usb_role);
    }

	if(IS_ERR_OR_NULL(ptn5150->partner)){

		ptn5150->partner = typec_register_partner(ptn5150->port, &desc);
		if(IS_ERR(ptn5150->partner)){
			dev_err(ptn5150->dev, "Failed to Register Partner %ld\n", 
					PTR_ERR(ptn5150->partner));

			return PTR_ERR(ptn5150->partner);
		}
	}

    return 0;
}

int ptn5150_detach(struct ptn5150* ptn5150)
{

    if(!ptn5150){
        return -EINVAL;
    }

    if(!IS_ERR_OR_NULL(ptn5150->partner)){
        typec_unregister_partner(ptn5150->partner);
        ptn5150->partner = NULL;
    }

	msleep(100); // workaround for usb-c registration issue
    typec_set_pwr_role(ptn5150->port, TYPEC_SOURCE);

    if(!IS_ERR_OR_NULL(ptn5150->role_switch)){
        usb_role_switch_set_role(ptn5150->role_switch, USB_ROLE_NONE);
    }

    typec_set_data_role(ptn5150->port, TYPEC_HOST);

    ptn5150->curr_status = PTN5150_CC_NOT_CONN;
    return 0;
}

static irqreturn_t ptn5150_irq_handler(int irq, void* data)
{
    uint8_t status;
    struct ptn5150* ptn5150 = (struct ptn5150*)data;


    if(ptn5150_read_reg(ptn5150, PTN5150_CABLE_INT_REG, &status) != 0){
        dev_err(ptn5150->dev, "IRQ: Error reading status\n");
        return IRQ_HANDLED;
    }

    if(status & PTN5150_INT_ATTACH){
        dev_dbg(ptn5150->dev, "IRQ: Attached\n");
        ptn5150_attach(ptn5150);
    }
    else if(status & PTN5150_INT_DETACH){
        dev_dbg(ptn5150->dev, "IRQ: Detached\n");
        ptn5150_detach(ptn5150);
    }

    return IRQ_HANDLED;
}


static int ptn5150_setup_irq(struct ptn5150* ptn5150)
{
    int ret;
    int irq;

    if(!ptn5150 || !ptn5150->dev){
        return -EINVAL;
    }

    //IRQ is not needed for DFP/Host only ports
    if(ptn5150->caps.data == TYPEC_PORT_DFP){
        return 0;
    }

    irq = ptn5150_get_irq(ptn5150);
    if(irq < 0){
        dev_err(ptn5150->dev, "Failed to get IRQ which is required for UFP/DRP\n");
        return -ENXIO;
    }

    ret = devm_request_threaded_irq(ptn5150->dev, irq, NULL,
                                    ptn5150_irq_handler, IRQF_ONESHOT,
                                    dev_name(ptn5150->dev), ptn5150);
    if(ret != 0){
        dev_err(ptn5150->dev, "IRQ Request Failed. err: %d\n", ret);
    }
    else{
        dev_dbg(ptn5150->dev, "IRQ Registered: %d\n", irq);
    }
    
    return ret;
}

static int ptn5150_parse_power_op_mode(struct ptn5150* ptn5150, 
                                       struct fwnode_handle* fwnode)
{
    const char* power_mode_str;
    int opmode;

    //Get advertised current limit
    if(fwnode_property_read_string(fwnode, "power-opmode", &power_mode_str) != 0){
        //power-op-mode not found in dt
        dev_err(ptn5150->dev, "No power-opmode prop in dt\n");
        return -EINVAL; 
    }

    opmode = typec_find_pwr_opmode(power_mode_str);

    //USB Power Delivery not supported 
    if(opmode  < 0 || opmode == TYPEC_PWR_MODE_PD){
        dev_err(ptn5150->dev, "Unsupported power-opmode %s in dt\n",
                 power_mode_str);

        return -EINVAL; 
    }

    ptn5150->pwr_opmode = opmode;

    return 0;
}


static int ptn5150_parse_port_type(struct ptn5150* ptn5150, 
                                   struct fwnode_handle* fwnode)
{
    const char* role_str;
    int ret;

    ret = fwnode_property_read_string(fwnode, "data-role", &role_str);

    //data-role not found in dt
    if(ret != 0){
        dev_err(ptn5150->dev, "No data-role prop in connector dt node\n");
        return -EINVAL; 
    }

    ret = typec_find_port_data_role(role_str);
    if(ret < 0){
        dev_err(ptn5150->dev, "Invalid data-role property %s in connector dt node\n",
                 role_str);

        return -EINVAL; 
    }

    dev_info(ptn5150->dev, "Port is %s \n", port_type_to_str(ret));

    ptn5150->caps.data = ret;

    //set Power Role based on Data Role
    switch(ptn5150->caps.data){
        case TYPEC_PORT_DFP:
            ptn5150->caps.type = TYPEC_PORT_SRC;
        break;
        case TYPEC_PORT_UFP:
            ptn5150->caps.type = TYPEC_PORT_SNK; 
        break;
        case TYPEC_PORT_DRD:
            ptn5150->caps.type = TYPEC_PORT_DRP; 
        break;
    }

    return 0;
}

static int ptn5150_parse_role_switch(struct ptn5150* ptn5150, 
                                     struct fwnode_handle* fwnode)
{

    if(!ptn5150 || !fwnode){
        return -EINVAL;
    }
        
    ptn5150->role_switch = fwnode_usb_role_switch_get(fwnode);

    if(IS_ERR_OR_NULL(ptn5150->role_switch)){

		//-EPROBE_DEFER if fine, do not print error
		if(PTR_ERR(ptn5150->role_switch) != -EPROBE_DEFER){
			dev_err(ptn5150->dev, "Failed to get usb_role_switch\n");
		}

        return PTR_ERR(ptn5150->role_switch);
    }

    return 0;
}

//Get the port capabilites from the device tree
static int ptn5150_get_capabilities(struct ptn5150* ptn5150,
                                    struct fwnode_handle* fwnode)
{
    int ret;

    if(!ptn5150){
        return -EINVAL;
    }

    ptn5150->caps.fwnode = fwnode;
    ptn5150->caps.revision = USB_TYPEC_REV_1_1;
    ptn5150->caps.prefer_role = TYPEC_NO_PREFERRED_ROLE;
    ptn5150->caps.driver_data = ptn5150;

    //Get default advertised current limit
    ret = ptn5150_parse_power_op_mode(ptn5150, fwnode);
    if(ret != 0){
        return ret;
    }

    ret = ptn5150_parse_port_type(ptn5150, fwnode);
    if(ret != 0){
        return ret;
    }

    return 0;
}

static int ptn5150_parse_dt(struct ptn5150* ptn5150)
{
    struct fwnode_handle* con_node; //connector node handle
    int ret;

    if(!ptn5150 || ! ptn5150->dev){
        return -EINVAL;
    }

    /*
        When "optional" is true the device may or may not be present.
        This prevents error messages when some revisions of a product
        use the ptn5150 part and some don't
    */
    ptn5150->optional = device_property_present(ptn5150->dev, "cti,optional");

    con_node = device_get_named_child_node(ptn5150->dev, "connector");
    if(!con_node){
        dev_err(ptn5150->dev, "No connector node found in dt\n");
        return -ENODEV;
    }

    //get the typec capabilities 
    ret = ptn5150_get_capabilities(ptn5150, con_node);
    if(ret != 0){
        dev_err(ptn5150->dev, "Failed to get port capabilities\n");
        fwnode_handle_put(con_node);
        return ret;
    }

    //If port is Host/DFP only no need to get role_switch
    if(!ptn5150_is_host_only(ptn5150)){
        ret = ptn5150_parse_role_switch(ptn5150, con_node);
    }

    fwnode_handle_put(con_node);

    return ret;
}

static int ptn5150_probe(struct i2c_client* client,
                         const struct i2c_device_id* id)
{
    int ret;
    struct ptn5150* ptn5150;
    
    ptn5150 = devm_kzalloc(&client->dev, sizeof(struct ptn5150), GFP_KERNEL);
    if(!ptn5150){
        return -ENOMEM;
    }

    ptn5150->client = client;
    ptn5150->dev = &client->dev;
    ptn5150->addr = client->addr;
    ptn5150->curr_status = PTN5150_CC_INVALID;


    dev_dbg(ptn5150->dev, "ptn5150 Driver Probe\n");

    ret = ptn5150_parse_dt(ptn5150);
    if(ret != 0){
        return ret;
    }

    //Initialize I2C interface 
    ptn5150->regmap = devm_regmap_init_i2c(ptn5150->client, &ptn5150_regmap_config);
    if(IS_ERR(ptn5150->regmap)) {
        dev_err(ptn5150->dev, "regmap init failed: (%ld)\n", PTR_ERR(ptn5150->regmap));
        return PTR_ERR(ptn5150->regmap);
    }
    dev_set_drvdata(ptn5150->dev, ptn5150);
    i2c_set_clientdata(client, ptn5150);

    //Probe for the device. Error out if not found and not marked optional
    if(!ptn5150_detect(ptn5150)){
        if(!ptn5150->optional){
            dev_err(ptn5150->dev, "PTN5150 USB-C Controller Not Detected\n");
            return -ENODEV;
        }
        else{
            //This is OK because the device was marked as optional
            dev_dbg(ptn5150->dev, "Optional PTN5150 USB-C Controller Not Detected\n");
            return 0;
        }
    }

    dev_dbg(ptn5150->dev, "PTN5150 USB PD Controller Detected\n");

    //configure the port to the mode determined from the dt
    ret = ptn5150_write_port_mode(ptn5150, ptn5150->caps.data);
    if(ret != 0){
        dev_err(ptn5150->dev, "Failed to set advertised current limit\n");
        return ret;
    }

    //set the default advertised current limit
    ret = ptn5150_write_cur_limit(ptn5150, ptn5150->pwr_opmode);
    if(ret != 0){
        dev_err(ptn5150->dev, "Failed to set advertised current limit\n");
        return ret;
    }

    //No need for registering or IRQ if host only
    if(ptn5150_is_host_only(ptn5150)){
        return 0;
    }

    ptn5150->port = typec_register_port(ptn5150->dev, &ptn5150->caps);
    if(IS_ERR_OR_NULL(ptn5150->port)) {
        dev_err(ptn5150->dev, "Failed to register typec port\n");
        return PTR_ERR(ptn5150->port);
    }

    //Check if cable was plugged in on bootup
    if(ptn5150_cable_detected(ptn5150)){
       ptn5150_attach(ptn5150); 
    }

    ret = ptn5150_setup_irq(ptn5150);
    if(ret != 0){
        dev_err(ptn5150->dev, "IRQ setup failed\n");
    }

    return ret;
}

static int ptn5150_remove(struct i2c_client* client)
{
    struct ptn5150* ptn5150;

    if(!client){
        return -EINVAL;
    }
    
    ptn5150 = i2c_get_clientdata(client);

    if(!ptn5150){
        return -EINVAL;
    }

    dev_dbg(ptn5150->dev, "Removing Driver\n");

    if(!IS_ERR_OR_NULL(ptn5150->partner)){
        typec_unregister_partner(ptn5150->partner);
        ptn5150->partner = NULL;
    }

    if(!IS_ERR_OR_NULL(ptn5150->role_switch)){
        usb_role_switch_put(ptn5150->role_switch);
    }

    if(!IS_ERR_OR_NULL(ptn5150->port)){
        typec_unregister_port(ptn5150->port);
    }

    return 0;
}


static const struct i2c_device_id ptn5150_id[] = {
    { "cti-ptn5150" },
    { },
};
MODULE_DEVICE_TABLE(i2c, ptn5150_id);

const struct of_device_id ptn5150_of_match[] = {
    { .compatible = "cti,ptn5150", },
    { },
};
MODULE_DEVICE_TABLE(of, ptn5150_of_match);

static struct i2c_driver ptn5150_i2c_driver = {
    .driver = {
        .owner = THIS_MODULE,
        .name = "cti-ptn5150",
        .of_match_table = ptn5150_of_match,
    },
    .probe = ptn5150_probe,
    .remove = ptn5150_remove,
    .id_table = ptn5150_id,
};

module_i2c_driver(ptn5150_i2c_driver);

MODULE_AUTHOR("Parker Newman <pnewman@connecttech.com>");
MODULE_DESCRIPTION("NXP PTN5150 USB-C Controller Driver");
MODULE_LICENSE("GPL v2");

