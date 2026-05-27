#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

/* Custom LBS UUID definitions */
static struct bt_uuid_128 lbs_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
);

static struct bt_uuid_128 lbs_char_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x00001524, 0x1212, 0xefde, 0x1523, 0x785feabcd123)
);

static char rx_buffer[64];
static struct bt_conn *current_conn;
static bool notifications_enabled;

/* Callback: Triggered when iPhone enables or disables notifications */
static void lbs_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
	printk("Notifications %s\n", notifications_enabled ? "enabled" : "disabled");
}

/* Callback: Triggered when you tap "Send 'Hello'" on your iPhone */
static ssize_t write_lbs_char(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			     const void *buf, uint16_t len, uint16_t offset, uint8_t flags)
{
	if (offset + len > sizeof(rx_buffer) - 1) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(rx_buffer + offset, buf, len);
	rx_buffer[offset + len] = '\0';

	printk("Received from iPhone: %s\n", rx_buffer);
	return len;
}

/* Callback: Triggered when iPhone reads data manually */
static ssize_t read_lbs_char(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    void *buf, uint16_t len, uint16_t offset)
{
	const char *msg = "nRF52 Alive!";
	return bt_gatt_attr_read(conn, attr, buf, len, offset, msg, strlen(msg));
}

/* Register GATT Service & Characteristics */
BT_GATT_SERVICE_DEFINE(lbs_svc,
	BT_GATT_PRIMARY_SERVICE(&lbs_uuid.uuid),
	BT_GATT_CHARACTERISTIC(&lbs_char_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
			       read_lbs_char, write_lbs_char, NULL),
	BT_GATT_CCC(lbs_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* Advertising Data Setup - Fixed little-endian layout */
static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, 
		0x23, 0xd1, 0xbc, 0xea, 0x5f, 0x78, 0x23, 0x15, 
		0xde, 0xef, 0x12, 0x12, 0x23, 0x15, 0x00, 0x00),
};

/* Scan Response Data */
static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* Connection Callbacks */
static void connected(struct bt_conn *conn, uint8_t err) {
	if (err) {
		printk("Connection failed (err %u)\n", err);
	} else {
		printk("Connected to iPhone!\n");
		current_conn = bt_conn_ref(conn);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
	printk("Disconnected from iPhone (reason %u)\n", reason);
	if (current_conn) {
		bt_conn_unref(current_conn);
		current_conn = NULL;
	}
	notifications_enabled = false;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

/* Function to push TX data notifications to the iPhone */
int send_sensor_notification(const char *data, uint16_t len)
{
	if (!current_conn || !notifications_enabled) {
		return -EACCES;
	}
	
	/* FIX: Index 2 targets the actual characteristic value attribute descriptor */
	return bt_gatt_notify(current_conn, &lbs_svc.attrs[2], data, len);
}

int main(void)
{
	int err;
	char tx_buffer[64]; // Ensure this array is explicitly sized
        // char tx_buffer[128];

	/* FIX: The lowercase parameter format matches your compatible driver string */
	const struct device *const mpu6050 = DEVICE_DT_GET_ONE(invensense_mpu6050);

	printk("Initializing Bluetooth Subsystem...\n");


	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	printk("Bluetooth initialized successfully.\n");

	/* MPU6050 Hardware Ready Verification Check */
	if (!device_is_ready(mpu6050)) {
		printk("Device MPU6050 is not ready!\n");
		return 0;
	}
	printk("MPU6050 Sensor driver loaded successfully.\n");


	struct bt_le_adv_param adv_param = {
		.options = BT_LE_ADV_OPT_CONN,
		.interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
		.interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
		.peer = NULL,
	};

	err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return 0;
	}

	printk("Advertising started! Waiting for iPhone...\n");

	/* Active execution frame loop inside main() */
        while (1) {
                /* Fetch sensor data updates every 250ms for smooth UI updates */
                k_sleep(K_MSEC(250));

                if (notifications_enabled) {
                        // ADDED: accel_z struct variable
                        struct sensor_value accel_x, accel_y, accel_z;

                        /* Request a hardware register sample read across I2C */
                        err = sensor_sample_fetch(mpu6050);
                        if (err) {
                                printk("Sensor sample fetch failed (err %d)\n", err);
                                continue;
                        }

                        /* Extract individual Accelerometer data channels */
                        sensor_channel_get(mpu6050, SENSOR_CHAN_ACCEL_X, &accel_x);
                        sensor_channel_get(mpu6050, SENSOR_CHAN_ACCEL_Y, &accel_y);
                        // ADDED: Read the Z axis channel
                        sensor_channel_get(mpu6050, SENSOR_CHAN_ACCEL_Z, &accel_z);

                        /* Convert Zephyr's internal struct layout into concrete floating values */
                        double x_val = sensor_value_to_double(&accel_x);
                        double y_val = sensor_value_to_double(&accel_y);
                        // ADDED: Convert Z axis
                        double z_val = sensor_value_to_double(&accel_z);

                        /* UPDATED: Package values into a short packet string including Z: "X:0.2,Y:-1.5,Z:9.8" */
                        snprintk(tx_buffer, sizeof(tx_buffer), "X:%.1f,Y:%.1f,Z:%.1f", x_val, y_val, z_val);
                        
                        err = send_sensor_notification(tx_buffer, strlen(tx_buffer));
                        if (!err) {
                        printk("Streaming data payload: %s\n", tx_buffer);
                        }
                }

        }


	return 0;
}
