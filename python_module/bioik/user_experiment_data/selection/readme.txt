First were removed the trajectories in collision 
which can not lead to collision free samples

Generate the trajectories by following ATERM_EXP.txt in the root readme folder
of the python_module

Then the following files are:

 * demo_in_collision.txt

Training set was selected first using

 * removed_from_training.txt

Testing set is thus

 * removed_from_training.txt - demo_in_collision.txt - remove3d_from_training.txt

