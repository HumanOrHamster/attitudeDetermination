# attitudeDetermination

## Summary
The following code provides attitude detmination of roll, pitch, and yaw in the 3,2,1 rotation sequence.

Harware implementation is through [MinIMU-9 v2 (L3GD20 and LSM303DLHC carrier)](https://www.pololu.com/catalog/product/1268) (discontinued) and the Arduino board. The MEMS IMU contains a 3-axis gyro, accelerometer, and magnetic compass. If the sensor that you are using contains these However the implementation can be genericly reused. 

The algorithm uses quaternion based states to keep track of the sensor's attitude. Direct euler's integration was avoided due to gimbal lock and DCM integration was avoided due to the number of states needed to be integrated and the normalization of the vectors within the DCM matrix. The quaternion vector contains only 4 states, and is easily normalized. 
That said there is an overhead associated with using the quaternion implementation in there is a need to convert to euler's representation (roll, pitch, yaw) to perform filtering.
The filtering implementation is a simple complementary filter between the gyro solution (smooth but error accumulates) and external ref solution (noisy but stable). In short, a complementary is nothing more than:

filtered_sol = prev_filtered_sol + K_gain * ( ref_sol - gyro_sol).

## Modifiable parts of the code
Since I have used a particular IMU MEMS sensor (mINimu-9 v2) to demonstrate this algorithm, if you intend to use a different sensor, then you will need to modify the following parameters and functions. 

// LSM303DLHC ACC and MAG Addresses
const int LSM303_ACC_ADDR       = 0b0011001;
const int LSM303_MAG_ADDR       = 0b0011110;

// MAG Register LSM303DLHC
const int CRA_REG_M         = 0x00;
const int CRB_REG_M         = 0x01;
const int MR_REG_M          = 0x02;
const int OUT_X_H_M         = 0x03; // starting read register for mag

// ACC Register LSM303DLHC
const int CTRL_REG4_A       = 0x23;
const int CTRL_REG1_A       = 0x20;
const int OUT_X_L_A         = 0x28; // starting read register for acc

// L3GD20 Address
const int L3GD20_GYRO_ADDR  = 0b1101011;

// GYRO Register L3GD20
const int CTRL_REG1_G     = 0x20;
const int CTRL_REG4_G     = 0x23;
const int OUT_X_L_G     = 0x28;


###  LSM303DLHC Accelerometer and Magnetic Compass


## Detail Algorithm Description


* [MinIMU-9 v2 (L3GD20 and LSM303DLHC carrier)](https://www.pololu.com/catalog/product/1268) (discontinued)
