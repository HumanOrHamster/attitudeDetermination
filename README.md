# attitudeDetermination

The following code provides attitude detmination of roll, pitch, and yaw in the 3,2,1 rotation sequence.
The algorithm uses quaternion based states to keep track of the sensor's attitude. Direct euler's integration was avoided due to gimbal lock and DCM integration was avoided due to the number of states needed to be integrated and the normalization of the vectors within the DCM matrix. The quaternion vector contains only 4 states, and is easily normalized. 
That said there is an overhead associated with using the quaternion implementation in there is a need to convert to euler's representation (roll, pitch, yaw) to perform filtering.
The filtering implementation is a simple complementary filter between the gyro solution (smooth but error accumulates) and external ref solution (noisy but stable). In short, a complementary is nothing more than,
filtered_sol = prev_filtered_sol + K_gain * ( ref_sol - gyro_sol).






https://www.pololu.com/product/1268

Algorithm uses quaternion based implementation to maintain angle states and integration.

Gravity and mag compass are used as ref fixes to bound gyro drift. Algorithm is intended to be robust agaisnt external mag interefence and external forces acting on the attitude sensor. 
