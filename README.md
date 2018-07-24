# attitudeDetermination

Attitude detmination algorithm using pololu imuv2

https://www.pololu.com/product/1268

Algorithm uses quaternion based implementation to maintain angle states and integration.

Gravity and mag compass are used as ref fixes to bound gyro drift. Algorithm is intended to be robust agaisnt external mag interefence and external forces acting on the attitude sensor. 
