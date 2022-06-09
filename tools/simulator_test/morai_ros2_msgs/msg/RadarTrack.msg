# This message relates only to FMCW radar.  
# All variables below are relative to the radar's frame of reference.
# This message is not meant to be used alone but as part of a stamped or array message.

# Object classifications (Additional vendor-specific classifications are permitted starting from 32000 eg. Car)

int8 uuid

                                            # Note: The z component of these fields is ignored for 2D tracking.
geometry_msgs/Point position                # x, y, z coordinates of the centroid of the object being tracked.
geometry_msgs/Vector3 velocity              # The velocity of the object in each spatial dimension.
geometry_msgs/Vector3 acceleration          # The acceleration of the object in each spatial dimension.
geometry_msgs/Vector3 size                  # The object size as represented by the radar sensor eg. length, width, height OR the diameter of an ellipsoid in the x, y, z, dimensions
                                            # and is from the sensor frame's view.
uint16 classification                       # An optional classification of the object (see above)

float32[] position_covariance              # Upper-triangle covariance about the x, y, z axes
float32[] velocity_covariance              # Upper-triangle covariance about the x, y, z axes
float32[] acceleration_covariance          # Upper-triangle covariance about the x, y, z axes
float32[] size_covariance                  # Upper-triangle covariance about the x, y, z axes

float32 amplitude

