'''
this file was modified from:

https://github.com/raulmur/evaluate_ate_scale.git
https://github.com/haliphinx/ORB-Mono-KITTI-Evaluation.git

please refer to the 2 github repositories above for more details
'''



import sys
import numpy as np
import matplotlib.pyplot as plt
import argparse

def gen_data(ground_time, res_time, ground_data):
	ground_time = ground_time
	res_time = res_time
	ground_data = ground_data

	time_mark = 0
	time = []

	data_1 = []

	for num in range(len(ground_data)):
		data_1.append(np.concatenate(([ground_time[num]], ground_data[num])))

	data_2 = []


	for num in range(len(res_time)):
		while not np.allclose(data_1[time_mark][0], res_time[num][0]):
			time_mark+=1
		data_2.append(data_1[time_mark])

	return data_2


def get_coo(data):
	points = [[],[],[]]
	for num in range(len(data)):
		points[0].append(data[num][4])
		points[1].append(data[num][8])
		points[2].append(data[num][12])
	return points


def get_points(data):
	points = [[],[],[]]
	for num in range(len(data)):
		points[0].append(data[num][1])
		points[1].append(data[num][2])
		points[2].append(data[num][3])
	return points


def align(model,data):
    """Align two trajectories using the method of Horn (closed-form).
    
    Input:
    model -- first trajectory (3xn)
    data -- second trajectory (3xn)
    
    Output:
    rot -- rotation matrix (3x3)
    trans -- translation vector (3x1)
    trans_error -- translational error per point (1xn)
    
    """

	# Center the data and model trajectories
    np.set_printoptions(precision=3,suppress=True)
    model_mean=[[model.mean(1)[0]], [model.mean(1)[1]], [model.mean(1)[2]]]
    data_mean=[[data.mean(1)[0]], [data.mean(1)[1]], [data.mean(1)[2]]]
    model_zerocentered = model - model_mean
    data_zerocentered = data - data_mean
    
	# Calculate the cross-covariance
    W = np.zeros( (3,3) )
    for column in range(model.shape[1]):
        W += np.outer(model_zerocentered[:,column],data_zerocentered[:,column])
    
	# Singular Value Decomposition(SVD) of the cross-covariance matrix
	U,d,Vh = np.linalg.linalg.svd(W.transpose())
    S = np.matrix(np.identity( 3 ))
    if(np.linalg.det(U) * np.linalg.det(Vh)<0):
        S[2,2] = -1
    rot = U*S*Vh # The rotation matrix

	# Compute the scaling factor "s"
    rotmodel = rot*model_zerocentered
    dots = 0.0
    norms = 0.0

    for column in range(data_zerocentered.shape[1]):
        dots += np.dot(data_zerocentered[:,column].transpose(),rotmodel[:,column])
        normi = np.linalg.norm(model_zerocentered[:,column])
        norms += normi*normi

    s = float(dots/norms)    

    print ("scale: %f " % s) 
    
	# compute translation
    trans = data_mean - s*rot * model_mean
    
	# Apply alignment transformation to the model
    model_aligned = s*rot * model + trans

	# Compute alignment error and translation error
    alignment_error = model_aligned - data 
    trans_error = np.sqrt(np.sum(np.multiply(alignment_error,alignment_error),0)).A[0]
        
    return rot,trans,trans_error, s


def plot_traj(ax, stamps, traj, style, colour, label)
    """
    Plot trajectory using matplotlib.

    Input:
    ax -- the plot
    stamps -- time stamps (1*n)
    traj -- trajectory (3*n)
    style -- line style
    colour -- line colour
    label -- plot legend
    """
    stamps.sort()
    # lag the timestamps and pair up
    interval = numpy.median([s-t for s, t in zip(stamps[1:],stamps[:-1])])
    x = []
    y = []
    last = stamps[0]
    for i in range(len(stamps)):
        if stamps[i] - last < 2*interval:
            x.append(traj[i][0])
            y.append(traj[i][1])
        elif len(x) > 0:
            ax.plot(x,y,style,colour=colour, label=label)
            label = ""
            x = []
            y = []
        last = stamps[i]
    if len(x) > 0:
        ax.plot(x, y, style, colour=colour, label=label)



if __name__ == '__main__':

  # PARSER COMMANDS
  # =====================================
    parser = argparse.ArgumentParser(description = 
    '''
    This script computes the Absolute Trajectory from the ground truth trajectory and estimated trajectory.
    '''
    )
    parser.add_argument('first_file', help='ground truth trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('second_file', help='estimated trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',default=0.0)
    parser.add_argument('--scale', help='scaling factor for the second trajectory (default: 1.0)',default=1.0)
    parser.add_argument('--max_difference', help='maximally allowed time difference for matching entries (default: 0.02)',default=0.02)
    parser.add_argument('--save', help='save aligned second trajectory to disk (format: stamp2 x2 y2 z2)')
    parser.add_argument('--save_associations', help='save associated first and aligned second trajectory to disk (format: stamp1 x1 y1 z1 stamp2 x2 y2 z2)')
    parser.add_argument('--plot', help='plot the first and the aligned second trajectory to an image (format: png)')
    parser.add_argument('--verbose', help='print all evaluation data (otherwise, only the RMSE absolute translational error in meters after alignment will be printed)', action='store_true')
    args = parser.parse_args()
    

  # PARAMETER INPUT
  # ======================================  
  # Path to the times.txt in KITTI dataset
	ground_time = np.loadtxt('/home/borui/Data/dataset/sequences/03/times.txt')
  
  # Path to the KeyFrameTrajectory.txt file
	res_time = np.loadtxt('/home/borui/Dev/COMP0130_22-23_Topic_03/Coursework_03/KeyFrameTrajectory.txt')
  
  # Path to the ground truth file
	ground_data = np.loadtxt('/home/borui/Data/dataset/sequences/03/GT03_KITTI.txt')
  # ========================================	
    
    data= gen_data(ground_time, res_time, ground_data)
	ground_points = np.asarray(get_coo(data))
	re_points = np.asarray(get_points(res_time))
	# print(type(ground_points))
	rot,trans,trans_error,s = align(re_points, ground_points)
	# print(rot)
	re_fpoints = s*rot*re_points+trans
	# print(re_fpoints[0])
	# print(trans_error)
	plt.scatter(ground_points[0], ground_points[2], s=0.1)
	plt.scatter(list(re_fpoints[0]), list(re_fpoints[2]), s=0.1, c='red')
	aa = list(re_fpoints[0])
	x = aa[0].tolist()
	aa = list(re_fpoints[2])
	y = aa[0].tolist()

	print ("compared_pose_pairs %d pairs"%(len(trans_error)))
	print ("absolute_translational_error.rmse %f m"%np.sqrt(np.dot(trans_error,trans_error) / len(trans_error)))
	print ("absolute_translational_error.mean %f m"%np.mean(trans_error))
	print ("absolute_translational_error.median %f m"%np.median(trans_error))
	print ("absolute_translational_error.std %f m"%np.std(trans_error))
	print ("absolute_translational_error.min %f m"%np.min(trans_error))
	print ("absolute_translational_error.max %f m"%np.max(trans_error))

	# for num in range(len(ground_points[0])):
	# 	plt.plot([ground_points[0][num], x[0][num]], [ground_points[2][num], y[0][num]], c = 'green')
	
	plt.show()


