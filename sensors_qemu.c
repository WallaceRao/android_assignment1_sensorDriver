
/*
 *          !!!!!!   SPECIFICATION     !!!!!
 *
 * This is the goldfish sensors HAL library for assignment two.
 * To make use of this library, the driver goldfish_sensors of
 * assignment one should make several changes:
 *
 * #1 support to enable certain sensor(s)
 *    In goldfish_sensor_write, accept command "setState%d".
 *    %d is the new sensors status that need to be performed.
 *    e.g., if it is 1, then only ID_ACCELERATION should be enabled
 *    if it is 7, all of the 3 sensors should be enabled.
 *    Driver store this value and check it whenever output
 *    sensors data to user space. If a sensor is disabled, driver will not
 *    output its value.
 *
 * #2 The output format of goldfish_sensor driver is modified.
 *   The data of different sensors is seperated by '|'
 *   e.g.
 *   for all sensors are enabled:
 *   acceleration:0,2511,0|compass:5632,1510,11033|gyroscope:0,0,0
 *   for ID_ACCELERATION is disabled and the other two are enabled:
 *   |compass:5632,1510,11033|gyroscope:0,0,0
 *
 * #3 Add support for command "list-sensors"
 *   In "goldfish_sensor_write", accept command "list-sensors"
 *   If driver receives this string, in next goldfish_sensor_read,
 *   it outputs sensors list to userspace. goldfish_sensors driver has
 *   3 sensors, so it will send integer 7, it means
 *   (SENSORS_ACCELERATION|SENSORS_COMPASS|SENSORS_GYROSCOPE).
 *
 * #4 Add a default delay in "goldfish_sensor_read" (MAY NOT NECESSARY)
 *   After Android is started, Android system will query sensor event at a very
 *   high frequency, this will cause the system very slow to respond to user input.
 *   To resolve this issue, add a delay in "goldfish_sensor_read".
 *   It is implemented by adding one line "msleep(100)".
 *
 *
 *
 *         !!!!!      HOW TO TEST      !!!!!
 *
 *  The test under Android 5.1 has been done, here are the steps:
 *  #1 Compile this file and generate sensors.goldfish.so
 *  #2 Launch emulator with specifing the kernek image which contains the goldfish_sensors driver
 *  #3 in host terminal, adb install "Sensors Multitool_v1.3.0_apkpure.com.apk" (install test app)
 *  #4 in host terminal, adb shell stop
 *  #5 in host terminal, adb root; adb remount
 *  #6 in adb shell, rm /system/lib/hw/sensors.goldfish.so
 *  #7 in host terminal,
 *     adb push [workdir]/out/target/product/generic/system/lib/hw/sensors.goldfish.so /system/lib/hw/
 *  #8 in adb shell, chmod 777 /dev/goldfish_sensor
 *  #9 in adb shell, echo 0 > /sys/fs/selinux/enforce
 *  #10 in host terminal, adb shell start
 *  #11 After android is started, open the app "Sensors Multitool",
 *      the ACCELERATION and MAGNETIC_FIELD seneors information could be
 *      seen, the xyz values match "virtual sensors" of the emulator.
 *      The gyroscope sensor values are always 0.
 */


/*
 * we connect with the emulator through /dev/goldfish_sensor
 */

#define DEVICENAME         "/dev/goldfish_sensor"

#define LOG_TAG "goldfishSensors"

#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <cutils/log.h>
#include <cutils/sockets.h>
#include <hardware/sensors.h>

#if 0
#define  D(...)  ALOGD(__VA_ARGS__)
#else
#define  D(...)  ((void)0)
#endif

#define  E(...)  ALOGE(__VA_ARGS__)

#include <hardware/qemud.h>

/** SENSOR IDS AND NAMES
 **/

#define MAX_NUM_SENSORS 3  // Only 3 sensors are considered

#define SUPPORTED_SENSORS  ((1<<MAX_NUM_SENSORS)-1)


#define  ID_BASE           SENSORS_HANDLE_BASE
#define  ID_ACCELERATION   (ID_BASE+0)
#define  ID_MAGNETIC_FIELD (ID_BASE+1)
#define  ID_GYROSCOPE      (ID_BASE+2)


#define  SENSORS_ACCELERATION    (1 << ID_ACCELERATION)
#define  SENSORS_COMPASS         (1 << ID_MAGNETIC_FIELD)
#define  SENSORS_GYROSCOPE       (1 << ID_GYROSCOPE)


#define  ID_CHECK(x)  ((unsigned)((x) - ID_BASE) < MAX_NUM_SENSORS)

#define  SENSORS_LIST  \
    SENSOR_(ACCELERATION,"acceleration") \
    SENSOR_(MAGNETIC_FIELD,"magnetic-field") \
    SENSOR_(GYROSCOPE,"gyroscope")

static const struct {
    const char*  name;
    int          id; } _sensorIds[MAX_NUM_SENSORS] =
{
#define SENSOR_(x,y)  { y, ID_##x },
    SENSORS_LIST
#undef  SENSOR_
};

static const char*
_sensorIdToName( int  id )
{
    int  nn;
    for (nn = 0; nn < MAX_NUM_SENSORS; nn++)
        if (id == _sensorIds[nn].id)
            return _sensorIds[nn].name;
    return "<UNKNOWN>";
}

static int
_sensorIdFromName( const char*  name )
{
    int  nn;

    if (name == NULL)
        return -1;

    for (nn = 0; nn < MAX_NUM_SENSORS; nn++)
        if (!strcmp(name, _sensorIds[nn].name))
            return _sensorIds[nn].id;

    return -1;
}

/* return the current time in nanoseconds */
static int64_t now_ns(void) {
    struct timespec  ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
}

/** SENSORS POLL DEVICE
 **
 ** This one is used to read sensor data from the hardware.
 ** We implement this by simply reading the data from the
 ** emulator through the QEMUD channel.
 **/

typedef struct SensorDevice {
    struct sensors_poll_device_1  device;
    sensors_event_t               sensors[MAX_NUM_SENSORS];
    uint32_t                      pendingSensors;
    uint32_t                      active_sensors;
    int                           fd;
    pthread_mutex_t               lock;
} SensorDevice;


static int devfd = -1;

/*
 * Always use openSensorsDevice/closeSensorsDevice to open/close device file!
 */

static int openSensorsDevice()
{
    if(devfd < 0)
        devfd = open(DEVICENAME, O_RDWR);
    D("%s: open fd = %d ", __FUNCTION__, devfd);
    return devfd;
}


static int closeSensorsDevice()
{
    if(devfd > 0)
        close(devfd);
    devfd = -1;
    D("%s: close fd = %d ", __FUNCTION__, devfd);
    return devfd;
}


/*
 * "sensor_device_get_fd_locked" is a wrapper of openSensorsDevice.
 */

static int sensor_device_get_fd_locked(SensorDevice* dev) {
    /* Create connection to /dev/goldfish_sensors */
    dev->fd = openSensorsDevice();
    return dev->fd;
}

/* Send a command to the sensors virtual device. |dev| is a device instance and
 * |cmd| is a zero-terminated command string. Return 0 on success, or error code
 * on failure. */
static int sensor_device_send_command_locked(SensorDevice* dev,
                                             const char* cmd) {
    int ret = 0;
    int fd = sensor_device_get_fd_locked(dev);
    if (fd < 0) {
        return fd;
    }

    char test[255] = {0};
    snprintf(test, sizeof(test), "%s",cmd);
    if (write(fd, test, strlen(test)) < 0) {
        ret = -errno;
        E("%s(fd=%d): ERROR: %s", __FUNCTION__, fd, strerror(errno));
    }
    return ret;
}


/* Pick up one pending sensor event. On success, this returns the sensor
 * id, and sets |*event| accordingly. On failure, i.e. if there are no
 * pending events, return -1.
 *
 * Note: The device's lock must be acquired.
 */
static int sensor_device_pick_pending_event_locked(SensorDevice* d,
                                                   sensors_event_t*  event)
{
    uint32_t mask = SUPPORTED_SENSORS & d->pendingSensors;
    if (mask) {
        uint32_t i = 31 - __builtin_clz(mask);
        d->pendingSensors &= ~(1U << i);
        *event = d->sensors[i];
        event->sensor = i;
        event->version = sizeof(*event);

        D("%s: %d [%f, %f, %f]", __FUNCTION__,
                i,
                event->data[0],
                event->data[1],
                event->data[2]);
        return i;
    }
    E("No sensor to return!!! pendingSensors=0x%08x", d->pendingSensors);
    // we may end-up in a busy loop, slow things down, just in case.
    usleep(100000);
    return -1;
}



/*
 * The value requred from driver is an integer, convert it to the readable
 * float value.
 */
float formatFloatFromInt(int a)
{
	return (a/256.0);
}



/*
 * Block until something is reported by the emulator.
 * On succes, return 0
 * and updates the |pendingEvents| and |pendingSensors| fields of |dev|.
 * On failure, return error code.
 *
 * Note: The device lock must be acquired when calling this function, and
 *       will still be held on return.
 */
static int sensor_device_poll_event_locked(SensorDevice* dev)
{
    D("%s: dev=%p", __FUNCTION__, dev);
    int fd = sensor_device_get_fd_locked(dev);
    if (fd < 0) {
        E("%s: Could not open device: %s", __FUNCTION__, strerror(-fd));
        return fd;
    }

    uint32_t new_sensors = 0U;
    sensors_event_t* events = dev->sensors;

    int ret = 0;
    for (;;) {

        /* read the next event */
        char buff[256] = {0};
        int len = read(fd, buff, sizeof(buff));

        if (len < 0) {
            ret = -errno;
            E("%s(fd=%d): Could not receive event data len=%d, errno=%d: %s",
              __FUNCTION__, fd, len, errno, strerror(errno));
            break;
        }

        buff[len] = 0;

        D("%s(fd=%d): received [%s]", __FUNCTION__, fd, buff);
        int32_t params[3];
        /* "acceleration:<x>,<y>,<z>" corresponds to an acceleration event */
        if (sscanf(buff, "acceleration:%d,%d,%d", params+0, params+1, params+2)
                == 3) {
            new_sensors |= SENSORS_ACCELERATION;
            events[ID_ACCELERATION].acceleration.x = formatFloatFromInt(params[0]);
            events[ID_ACCELERATION].acceleration.y = formatFloatFromInt(params[1]);
            events[ID_ACCELERATION].acceleration.z = formatFloatFromInt(params[2]);
            events[ID_ACCELERATION].type = SENSOR_TYPE_ACCELEROMETER;
        }

        /* "compass:<x>,<y>,<z>" is sent */
        char *pos = strchr(buff, '|'); // compass data follows the first seperator
        if(!pos)
        {
            E("%s, no first seperator '|' found", __FUNCTION__);
            goto out;

        }
        pos++;
        if (sscanf(pos, "compass:%d,%d,%d", params+0, params+1, params+2)
                == 3) {
            new_sensors |= SENSORS_COMPASS;
            events[ID_MAGNETIC_FIELD].magnetic.x = formatFloatFromInt(params[0]);
            events[ID_MAGNETIC_FIELD].magnetic.y = formatFloatFromInt(params[1]);
            events[ID_MAGNETIC_FIELD].magnetic.z = formatFloatFromInt(params[2]);
            events[ID_MAGNETIC_FIELD].type = SENSOR_TYPE_MAGNETIC_FIELD;
        }

        /* "gyroscope:<x>,<y>,<z>" is sent for the params of the gyroscope field */
        pos = strchr(pos, '|');  // compass data follows the second seperator
        if(!pos)
        {
            E("%s, no second seperator '|' found", __FUNCTION__);
            goto out;

        }
        pos++;
        if (sscanf(pos, "gyroscope:%d,%d,%d", params+0, params+1, params+2)
                == 3) {
            new_sensors |= SENSORS_GYROSCOPE;
            events[ID_GYROSCOPE].gyro.x = formatFloatFromInt(params[0]);
            events[ID_GYROSCOPE].gyro.y = formatFloatFromInt(params[1]);
            events[ID_GYROSCOPE].gyro.z = formatFloatFromInt(params[2]);
            events[ID_GYROSCOPE].type = SENSOR_TYPE_GYROSCOPE;
        }
        if (new_sensors) {
                goto out;
        }
        E("huh ? unsupported command");
        // Break the loop in any case.
        break;
    }
out:
    if (new_sensors) {
        dev->pendingSensors |= new_sensors;
    }
    return ret;
}



/** SENSORS POLL DEVICE FUNCTIONS **/

static int sensor_device_close(struct hw_device_t* dev0)
{
    SensorDevice* dev = (void*)dev0;
    // Assume that there are no other threads blocked on poll()
    if (dev->fd >= 0) {
        dev->fd = closeSensorsDevice();
    }
    pthread_mutex_destroy(&dev->lock);
    free(dev);
    return 0;
}

/*
 * Return an array of sensor data. This function blocks until there is something
 * is reported by driver. On success, it will write the events into the
 * |data| array, which contains |count| items. The function assigns "number"
 * events, which shall never be greater than |count|.
 *
 * Note that according to the sensor HAL [1], it shall never return 0!
 * So always return count and this will be fine as caller should check the event.
 *
 * [1] http://source.android.com/devices/sensors/hal-interface.html
 */
static int sensor_device_poll(struct sensors_poll_device_t *dev0,
                              sensors_event_t* data, int count)
{
    SensorDevice* dev = (void*)dev0;
    D("%s: dev=%p data=%p count=%d ", __FUNCTION__, dev, data, count);

    if (count <= 0) {
        return -EINVAL;
    }

    pthread_mutex_lock(&dev->lock);
    if (!dev->pendingSensors) {
        /* Block until there are somethind reported by driver. */
        int ret = sensor_device_poll_event_locked(dev);
        if (ret < 0 || !dev->pendingSensors) {
            goto out;
        }
    }
    /* Now read as many pending events as needed. */
    int i;
    for (i = 0; i < count; i++)  {
        if (!dev->pendingSensors) {
            break;
        }
        int ret = sensor_device_pick_pending_event_locked(dev, data);
        if (ret < 0) {
            break;
        }
        data++;
    }
out:
    pthread_mutex_unlock(&dev->lock);
    //always return count and this will be fine as the caller should check every event.
    return count;
}



/*
 * Enable/disable a specific sensor, handle is the ID of the sensor,
 * valid IDs are 1,2,and 3.
 * If the sensor state does not equal to "enabled", update all sensors
 * state by sending "setState%d" to driver.
 */

static int sensor_device_activate(struct sensors_poll_device_t *dev0,
                                  int handle,
                                  int enabled)
{
    SensorDevice* dev = (void*)dev0;
    D("%s: handle=%s (%d) enabled=%d", __FUNCTION__,
        _sensorIdToName(handle), handle, enabled);

    /* Sanity check */
    if (!ID_CHECK(handle)) {
        E("%s: bad handle ID", __FUNCTION__);
        return -EINVAL;
    }

    /* Exit early if sensor is already enabled/disabled. */
    char mask = (1U << handle);
    char sensors = enabled ? mask : 0;

    pthread_mutex_lock(&dev->lock);

    char active = dev->active_sensors;
    char new_sensors = (active & ~mask) | (sensors & mask);
    char changed = active ^ new_sensors;

    int ret = 0;
    if (changed) {
        char command[64]= {0};
        snprintf(command,
                 sizeof(command),
                 "setState%d",
                 new_sensors);
        /* Send command to the emulator. */
        ret = sensor_device_send_command_locked(dev, command);
        if (ret < 0) {
            E("%s: when sending command errno=%d: %s", __FUNCTION__, -ret,
              strerror(-ret));
        } else {
            dev->active_sensors = new_sensors;
        }
    }
    pthread_mutex_unlock(&dev->lock);
    return ret;
}




/** MODULE REGISTRATION SUPPORT
 **
 ** This is required so that hardware/libhardware/hardware.c
 ** will dlopen() this library appropriately.
 **/

/*
 * the following is the list of all supported sensors.
 * this table is used to build sSensorList declared below
 * according to which hardware sensors are reported as
 * available from the emulator (see get_sensors_list below)
 *
 * note: numerical values for maxRange/resolution/power for
 *       all sensors but light, pressure and humidity were
 *       taken from the reference AK8976A implementation
 */
static const struct sensor_t sSensorListInit[] = {
         { .name       = "Goldfish 3-axis Accelerometer",
          .vendor     = "Yonghui Rao",
          .version    = 1,
          .handle     = ID_ACCELERATION,
          .type       = SENSOR_TYPE_ACCELEROMETER,
          .maxRange   = 2.8f,
          .resolution = 1.0f/4032.0f,
          .power      = 3.0f,
          .reserved   = {}
        },
        { .name       = "Goldfish 3-axis Magnetic field sensor",
          .vendor     = "Yonghui Rao",
          .version    = 1,
          .handle     = ID_MAGNETIC_FIELD,
          .type       = SENSOR_TYPE_MAGNETIC_FIELD,
          .maxRange   = 2000.0f,
          .resolution = 1.0f,
          .power      = 6.7f,
          .reserved   = {}
        },
        { .name       = "Goldfish GYROSCOPE sensor",
          .vendor     = "Yonghui Rao",
          .version    = 1,
          .handle     = ID_GYROSCOPE,
          .type       = SENSOR_TYPE_GYROSCOPE,
          .maxRange   = 360.0f,
          .resolution = 1.0f,
          .power      = 9.7f,
          .reserved   = {}
        },
    };

static struct sensor_t  sSensorList[MAX_NUM_SENSORS];



/*
 * Get avaliable sensors list, first send command ""list-sensors"
 * to driver, then read the output of the driver. Driver will return
 * a mask of all sensors, in this case, goldfish_sensor driver will always
 * send 7(SENSORS_ACCELERATION|SENSORS_COMPASS| SENSORS_GYROSCOPE).
 * This method will return the list of the 3 sensors.
 */

static int sensors__get_sensors_list(struct sensors_module_t* module __unused,
        struct sensor_t const** list)
{
    int  fd = openSensorsDevice();
    char buffer[255] = {0};
    int  nn, count;
    int  ret = 0;

    if (fd < 0) {
        E("%s: no device connection", __FUNCTION__);
        goto out;
    }
    char command[255] = {0};

    snprintf(command, sizeof(command), "list-sensors");
    ret = write(fd, command, strlen(command));
    if (ret < 0) {
        E("%s: could not query sensor list, fd:%d, : %s", __FUNCTION__,fd,
          strerror(errno));
        goto out;
    }
    ret = read(fd, buffer, sizeof(buffer)-1);
    if (ret < 0) {
        E("%s: could not receive sensor list: %s", __FUNCTION__,
          strerror(errno));
        goto out;
    }
    int mask = 0;
    buffer[ret] = 0;
      E("%s: receive buffer:%s", __FUNCTION__, buffer);

    if (sscanf(buffer, "%d", &mask) != 1) {
        E("%s: sscanf error, buffer:%s", __FUNCTION__, buffer);
    }

    count = 0;
    for (nn = 0; nn < MAX_NUM_SENSORS; nn++) {
        if (((1 << nn) & mask) == 0)
            continue;
        sSensorList[count++] = sSensorListInit[nn];
    }
    E("%s: returned %d sensors (mask=%d)", __FUNCTION__, count, mask);
    *list = sSensorList;

    ret = count;
out:
    if (fd >= 0) {
        closeSensorsDevice();
    }
    return ret;
}


/*
 * According to https://source.android.com/devices/sensors/hal-interface
 * this API is only used in HAL version 1.0, as the goldfish driver
 * already has a default delay which seems works fine, no need to provide
 * this function to application layer.
 */

static int poll__setDelay(struct sensors_poll_device_1 *dev,
            int handle, int64_t ns)
{
    // TODO
    return 0;
}


/*
 * Open sensors, generate a "SensorDevice" and set values
 */

static int
open_sensors(const struct hw_module_t* module,
             const char*               name,
             struct hw_device_t*      *device)
{
    int  status = -EINVAL;

    D("%s: name=%s", __FUNCTION__, name);

    if (!strcmp(name, SENSORS_HARDWARE_POLL)) {
        SensorDevice *dev = malloc(sizeof(*dev));

        memset(dev, 0, sizeof(*dev));

        dev->device.common.tag     = HARDWARE_DEVICE_TAG;
        dev->device.common.version = SENSORS_DEVICE_API_VERSION_1_0;
        dev->device.common.module  = (struct hw_module_t*) module;
        dev->device.common.close   = sensor_device_close;
        dev->device.poll           = sensor_device_poll;
        dev->device.activate       = sensor_device_activate;
        dev->device.setDelay       = poll__setDelay;
        // by default, assume all sensors are enabled, since driver also enables all sensors by default.
        dev->active_sensors = SENSORS_ACCELERATION|SENSORS_COMPASS|SENSORS_GYROSCOPE;
        dev->fd = -1;
        pthread_mutex_init(&dev->lock, NULL);

        *device = &dev->device.common;
        status  = 0;
    }
    return status;
}


static struct hw_module_methods_t sensors_module_methods = {
    .open = open_sensors
};

struct sensors_module_t HAL_MODULE_INFO_SYM = {
    .common = {
        .tag = HARDWARE_MODULE_TAG,
        .version_major = 1,
        .version_minor = 0,
        .id = SENSORS_HARDWARE_MODULE_ID,
        .name = "Goldfish SENSORS Module",
        .author = "Yonghui Rao",
        .methods = &sensors_module_methods,
    },
    .get_sensors_list = sensors__get_sensors_list
};
