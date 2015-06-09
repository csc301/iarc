#include "urg_c/urg_sensor.h"
#include "urg_c/urg_utils.h"
#include "open_urg_sensor.h"
#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <netdb.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <malloc.h>
#include <arpa/inet.h>
#include <sys/time.h>
#include <unistd.h>
#include <libgen.h>
#include <csm/csm_all.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>

#define SERVPORT 6666
#define PI 3.14159265

void set_sm_params(struct sm_params *params, long min_distance, long max_distance);

LDP set_laser_data(long *data, long min_distance, long max_distance, long time_stamp);

int set_term(int fd, int nSpeed, int nBits, char nEvent, int nStop);

int main(int argc, char *argv[])
{
	/********************** For Socket *********************/
	int sockfd;
	ssize_t sendbytes, recvbytes;
	struct hostent *host;
	struct sockaddr_in address, server_address;
	int len;
    char *recvBufferPtr = NULL;
    char *sendBufferPtr = NULL;
    char order;
	float buffer[5];
    int select_flag;
    struct timeval timeout;
    fd_set readfds, readfds_tmp;

	if(argc < 2) {
		fprintf(stderr, "Please enter the server's hostname!\n");
		exit(1);
	}
	/* 地址解析 */
	if((host = gethostbyname(argv[1])) == NULL) {
		perror("gethostbyname");
		exit(1);
	}

	/* 创建 socket */
	if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) == -1) {
		perror("socket");
		exit(1);
	}

	/* 设置 sockaddr_in 结构体中相关参数 */
	address.sin_family = AF_INET;
	address.sin_port = htons(SERVPORT);
	address.sin_addr = *((struct in_addr *)host->h_addr);
	bzero(&(address.sin_zero), 8);
	len = sizeof(address);
    server_address.sin_family = AF_INET;
    server_address.sin_port = htons(SERVPORT);
    server_address.sin_addr.s_addr = inet_addr("10.42.0.1");

    bind(sockfd, (struct sockaddr *)&server_address, len);
    listen(sockfd, 1);

	FD_ZERO(&readfds);
	FD_SET(sockfd, &readfds);



	/********************** For HOKUYO Laser Scanner *********************/
	/* 获取 Hokuyo 激光雷达数据 */
	urg_t urg;
	long laser_type = 0;
	long *data = NULL;
	size_t datasize;
	long min_distance, max_distance;
	long time_stamp;
	long head[3];
	int n;

	if (open_urg_sensor(&urg, argc, argv) < 0) {
		perror("open_urg_sensor()");
		exit(1);
	}

	//	type		urg_max_data_size(&urg)		datasize
	//	UTM-30LX	????				        ???? Bytes
	//	URG-04LX	726                         2904 Bytes
	
	datasize = urg_max_data_size(&urg) * sizeof(data[0]);
	if(datasize == 2904) {
		laser_type = 4;
	}
	else {
		perror("unknown laser type");
		exit(1);
	}

	data = (long *)malloc(datasize);
	if (!data) {
		perror("urg_max_index()");
		exit(1);
	}

	urg_distance_min_max(&urg, &min_distance, &max_distance);
    printf("set hokuyo laser!\n");



    /********************** For Serial Communication *********************/
    int nread, i;
    int serialfd;
    struct termios oldtio;
    serialfd = open("/dev/ttyO3", O_RDWR);
    if(-1 == serialfd) {
        perror("open serial error!");
        exit(1);
    }
    /* save the attributes about terminals */
    if(tcgetattr(serialfd, &oldtio) != 0) {
        perror("SetupSerial");
        return -1;
    }
    if((i = set_term(serialfd, 115200, 8, 'N', 1)) < 0) {
        perror("set_term error");
        exit(1);
    }
    short send_buffer[3];
    float read_buffer[3];
    struct timeval serial_timeout;
    fd_set serial_readfds, serial_testfds;
    FD_ZERO(&serial_readfds);
    FD_SET(serialfd, &serial_readfds);



	/********************** For CSM *********************/
	struct sm_params params;
	struct sm_result result;

	set_sm_params(&params, min_distance, max_distance);

	LDP laser_ref, laser_sens;

	/* Read first scan */
	urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
	n = urg_get_distance(&urg, data, &time_stamp);
	while (n <= 0) {
		printf("urg_get_distance: %s\n", urg_error(&urg));
		free(data);
		urg_close(&urg);
		close(sockfd);
		exit(1);
	}

	laser_ref = set_laser_data(data, min_distance, max_distance, time_stamp);

	if(!ld_valid_fields(laser_ref)) {
		sm_error("Invalid laser data in first scan.\n");
		return -2;
	}

	/* For the first scan, set estimate = odometry */
	copy_d(laser_ref->odometry, 3, laser_ref->estimate);



    /* PID Parameters */
    order = 0;
    short threshold = 100;
    double kp = 1.0;
    double kd = 0.2;
    double kpv = 4.0;
    double kptheta = 400.0;
    double theta = 0.0;
    double lastTheta = 0.0;
    float positionSet[3];
    double ex, ey, etheta;
    double thetaComp = 0.0;
    ex = 0.0;
    ey = 0.0;
    etheta = 0.0;
    double positionRef[3];
    recvBufferPtr = (char *)malloc(39);
    sendBufferPtr = (char *)malloc(51);
    double yawOffSet = 0.0;
    int initCounter = 0;

    /* velocity estimate */
    double velocity[2];
    velocity[0] = 0.0;
    velocity[1] = 0.0;
    double velocityPrior[2];
    velocityPrior[0] = 0.0;
    velocityPrior[1] = 0.0;
    double control[2];
    control[0] = 0.0;
    control[1] = 0.0;
    double p = 0.0;
    double pPrior = 0.0;
    double kg = 0.0;

    printf("run main loop!\n");

	while(order < 2) {

        /* Read data from MCU */
        lastTheta = theta;
        char orderToMCU = 0xf1;
        int data_counter;
        data_counter = write(serialfd, &orderToMCU, 1);
        int serial_fd;
        serial_testfds = serial_readfds;
        serial_timeout.tv_sec = 0;
        serial_timeout.tv_usec = 8000; // 8ms

        select_flag = select(FD_SETSIZE, &serial_testfds, (fd_set *)NULL, (fd_set *)NULL, &serial_timeout);
        switch(select_flag) {
        case 0:
            break;
        case -1:
            perror("select");
			free(data);
            free(recvBufferPtr);
            free(sendBufferPtr);
			urg_close(&urg);
			close(sockfd);
            exit(1);
        default:
            for(serial_fd = 0; serial_fd < FD_SETSIZE; serial_fd++) {
                if(FD_ISSET(serial_fd, &serial_testfds)) {
                    data_counter = read(serial_fd, read_buffer, 12);
                    double thetaTmp = (double)(read_buffer[0]);


                    /* compensating errors in thera */
                    if(thetaTmp < 1.58476)
                        thetaComp = 0.2283 * thetaTmp * thetaTmp * thetaTmp - 0.4008 * thetaTmp * thetaTmp + 1.053 * thetaTmp;
                    else if(thetaTmp < 2.15374)
                        thetaComp = 1.0563 * thetaTmp - 0.1032;
                    else if(thetaTmp < 5.52396)
                        thetaComp = -0.0222 * thetaTmp * thetaTmp * thetaTmp + 0.1614 * thetaTmp * thetaTmp + 0.7873 * thetaTmp - 0.0456;
                    else
                        thetaComp = 1.0435 * thetaTmp - 0.2732;
                    if(thetaComp < 0)
                        thetaComp = 0;
                    else if(thetaComp > 6.2831853)
                        thetaComp = 6.2831853;
               

                    if(initCounter == 0) {
                        initCounter++;
                        yawOffSet = thetaComp;
                    }
                    theta = thetaComp - yawOffSet;
                    if(theta >= PI) {
                        theta -= (2 * PI);
                    }
                    else if(theta < -PI) {
                        theta += (2 * PI);
                    }
                }
            }
            break;
        }



        /* Read data from hokoyu laser */
		urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
		n = urg_get_distance(&urg, data, &time_stamp);
		if (n <= 0) {
			printf("urg_get_distance: %s\n", urg_error(&urg));
			free(data);
            free(recvBufferPtr);
            free(sendBufferPtr);
			urg_close(&urg);
			close(sockfd);
			exit(1);
		}

		// The Canonical Scan Matching Algorithm
		laser_sens = set_laser_data(data, min_distance, max_distance, time_stamp);
		if(!ld_valid_fields(laser_sens)) {
			sm_error("Invalid laser data. \n");
			continue;
		}

		params.laser_ref = laser_ref;
		params.laser_sens = laser_sens;

		/* Set first guess as the difference in odometry */
		if(	any_nan(params.laser_ref->odometry,3) ||  
			any_nan(params.laser_sens->odometry,3) ) {
				sm_error("The 'odometry' field is set to NaN so I don't know how to get an initial guess. I usually use the difference in the odometry fields to obtain the initial guess.\n");
				sm_error("  laser_ref->odometry = %s \n",  friendly_pose(params.laser_ref->odometry) );
				sm_error("  laser_sens->odometry = %s \n", friendly_pose(params.laser_sens->odometry) );
				sm_error(" I will quit it here. \n");
				return -3;
		}
		
		double odometry[3];
		pose_diff_d(laser_sens->odometry, laser_ref->odometry, odometry);
        odometry[2] = theta - lastTheta;
		double ominus_laser[3], temp[3];
		ominus_d(params.laser, ominus_laser);
		oplus_d(ominus_laser, odometry, temp);
		oplus_d(temp, params.laser, params.first_guess);

		/* Do the actual work */
        double t[3];
        t[0] = 0.0;
        t[1] = 0.0;
        t[2] = 0.0;
        // sm_icp(&params, &result);
        new_sm_icp(&params, &result, &t, (double)read_buffer[1], (double)read_buffer[2]);



        /* receive data from PC */
		int fd;
        readfds_tmp = readfds;
		timeout.tv_sec = 0;
		timeout.tv_usec = 5000; // 5ms

		select_flag = select(FD_SETSIZE, &readfds_tmp, (fd_set *)NULL, (fd_set *)NULL, &timeout);

		switch(select_flag) {
		case 0:
			break;
		case -1:
			perror("select");
			ld_free(laser_ref);
			free(data);
            free(recvBufferPtr);
            free(sendBufferPtr);
			urg_close(&urg);
			close(sockfd);
			exit(1);
		default:
			for(fd = 0; fd < FD_SETSIZE; fd++) {
                if(FD_ISSET(fd, &readfds_tmp)) {
                    //* receive order from pc */
                    recvbytes = recvfrom(fd, recvBufferPtr, 39, 0, (struct sockaddr *)&address, &len);
                    memcpy(&order, recvBufferPtr, 1);
                    memcpy(&threshold, recvBufferPtr + 1, 2);
                    memcpy(&kp, recvBufferPtr + 3, 8);
                    memcpy(&kd, recvBufferPtr + 11, 8);
                    memcpy(&kptheta, recvBufferPtr + 19, 8);
                    positionRef[0] = (laser_ref->estimate)[0];
                    positionRef[1] = (laser_ref->estimate)[1];
                    positionRef[2] = (laser_ref->estimate)[2];
                    velocity[0] = 0.0;
                    velocity[1] = 0.0;
                    p = 0.0;
                }
			}
			break;
		}
        // printf("order from PC: %d\n", order);
        // printf("threshold: %d\n", threshold);
        // printf("kp: %f\n", kp);
		if(!result.valid) {
			sm_error("One ICP matching failed. Because I process recursively, I will ignore this result!\n");
            continue;
			// ld_free(laser_ref);
			// free(data);
			// urg_close(&urg);
			// close(sockfd);
			// return 2;
		} else {
			/* Add the result to the previous estimate */
			oplus_d(laser_ref->estimate, result.x, laser_sens->estimate);
            (laser_sens->estimate)[2] = theta;
			ld_free(laser_ref);
			laser_ref = laser_sens;
            if(order == 0) {
                ex = 0.0;
                ey = 0.0;
                (laser_ref->estimate)[0] = 0.0;
                (laser_ref->estimate)[1] = 0.0;
                (laser_ref->estimate)[2] = 0.0;
                (laser_sens->estimate)[0] = 0.0;
                (laser_sens->estimate)[1] = 0.0;
                (laser_sens->estimate)[2] = 0.0;
                initCounter = 0;
            }
            else if(order == 1) {

                // * velocity estimate */
                // velocityPrior[0] = velocity[0] + 0.286 * (control[0] * cos(theta) - control[1] * sin(theta));
                // velocityPrior[1] = velocity[1] + 0.286 * (control[0] * sin(theta) + control[1] * cos(theta));
                velocityPrior[0] = velocity[0] + 0.286 * control[0];
                velocityPrior[1] = velocity[1] + 0.286 * control[1];
                pPrior = p + 25;
                kg = pPrior / (pPrior + 4);
                // velocity estimated value (cm/s)
                velocity[0] = velocityPrior[0] + kg * (1000 * (result.x)[0] - velocityPrior[0]);
                velocity[1] = velocityPrior[1] + kg * (1000 * (result.x)[1] - velocityPrior[1]);
                p = (1 - kg) * pPrior;

                memcpy(&positionSet, recvBufferPtr + 27, 12);
                double offsetInWorld[2];
                offsetInWorld[0] = positionRef[0] - (laser_sens->estimate)[0];
                offsetInWorld[1] = positionRef[1] - (laser_sens->estimate)[1];
                double velocityRef[2];
       velocityRef[0] = 100 * kp * ((double)(positionSet[0]) + offsetInWorld[0] * cos(theta) + offsetInWorld[1] * sin(theta)) - kd * velocity[0];
       velocityRef[1] = 100 * kp * ((double)(positionSet[1]) + offsetInWorld[1] * cos(theta) - offsetInWorld[0] * sin(theta)) - kd * velocity[1];

                if(velocityRef[0] > 75)
                    velocityRef[0] = 75;
                if(velocityRef[0] < -75)
                    velocityRef[0] = -75;
                if(velocityRef[1] > 75)
                    velocityRef[1] = 75;
                if(velocityRef[1] < -75)
                    velocityRef[1] = -75;

                ex = kpv * (velocityRef[0] - velocity[0]);
                ey = kpv * (velocityRef[1] - velocity[1]);

                etheta = ((double)(positionSet[2]) + positionRef[2] - (laser_sens->estimate)[2]);
                if(etheta > PI) {
                    etheta -= (2 * PI);
                }
                else if(etheta < -PI) {
                    etheta += (2 * PI);
                }
                etheta *= kptheta;
            }
		}

        send_buffer[0] = (short)(ex);
        send_buffer[1] = (short)(ey);
        send_buffer[2] = (short)(etheta);
        if(send_buffer[0] > threshold)
            send_buffer[0] = threshold;
        if(send_buffer[0] < (-1 * threshold))
            send_buffer[0] = (-1 * threshold);
        if(send_buffer[1] > threshold)
            send_buffer[1] = threshold;
        if(send_buffer[1] < (-1 * threshold))
            send_buffer[1] = (-1 * threshold);
        if(send_buffer[2] > threshold)
            send_buffer[2] = threshold;
        if(send_buffer[2] < (-1 * threshold))
            send_buffer[2] = (-1 * threshold);

        control[0] = (double)send_buffer[0];
        control[1] = (double)send_buffer[1];

        orderToMCU = 0xf2;
        data_counter = write(serialfd, &orderToMCU, 1);
        data_counter = write(serialfd, send_buffer, 6);

        //* send data to pc */
        buffer[0] = (float)time_stamp;
        buffer[1] = (float)(laser_sens->estimate)[0];
        buffer[2] = (float)(laser_sens->estimate)[1];
        buffer[3] = (float)(laser_sens->estimate)[2];
        buffer[4] = read_buffer[0];
        memcpy(sendBufferPtr, &order, 1);
        memcpy(sendBufferPtr + 1, &buffer, 20);
        memcpy(sendBufferPtr + 21, &thetaComp, 8);
        memcpy(sendBufferPtr + 29, &send_buffer, 6);
        memcpy(sendBufferPtr + 35, &velocity, 16);
        sendbytes = sendto(sockfd, sendBufferPtr, 51, 0, (struct sockaddr *)&address, len);
	}
	close(sockfd);
	free(data);
    free(recvBufferPtr);
    free(sendBufferPtr);
	urg_close(&urg);
	ld_free(laser_ref);
    tcflush(serialfd, TCIFLUSH);

    if((tcsetattr(serialfd, TCSANOW, &oldtio)) != 0) {
        perror("com set error");
        return -1;
    }
    close(serialfd);

	return 0;
}

void set_sm_params(struct sm_params *params, long min_distance, long max_distance)
{
	/** Maximum angular displacement between scans (deg) */
	params->max_angular_correction_deg = 15.0;
	/** Maximum translation between scans (m) */
	params->max_linear_correction = 0.2;
	/** When we had enough */
	params->max_iterations = 800;
	/** A threshold for stopping (m) */
	params->epsilon_xy = 0.0005;
	/** A threshold for stopping (rad) */
	params->epsilon_theta = 0.0005;
	/** Maximum distance for a correspondence to be valid (m) */
	params->max_correspondence_dist = 0.2;
	/** Noise in the scan (m) */
	params->sigma = 0.01;
	/** Use smart tricks for finding correspondences */
	params->use_corr_tricks = 1;

	/** Restart: Restart if error is over threshold */
	params->restart = 1;
	/** Restart: Threshold for restarting */
	params->restart_threshold_mean_error = 0.01;
	/** Restart: Displacement for restarting (m) */
	params->restart_dt = 0.01;
	/** Restart: Displacement for restarting (rad) */
	params->restart_dtheta = 0.0262;

	/** Max distance for staying in the same clustering */
	params->clustering_threshold = 0.05;
	/** Number of neighbour rays used to estimate the orientation */
	params->orientation_neighbourhood = 3;
	/** If 1, it's PLICP */
	params->use_point_to_line_distance = 1;

	/** Discard correspondences based on the angles */
	params->do_alpha_test = 0;
	params->do_alpha_test_thresholdDeg = 20.0;

	/** Percentage of correspondences to consider */
	params->outliers_maxPerc = 0.95;
	params->outliers_adaptive_order = 0.7;
	params->outliers_adaptive_mult = 2.0;

	/**  */
	params->do_visibility_test = 0;
	/** No two points in laser_sens can have the same corr */
	params->outliers_remove_doubles = 1;
	/** If 1, computes the covariance of ICP */
	params->do_compute_covariance = 0;
	/** Checks that find_correspondences_tricks gives the right answer */
	params->debug_verify_tricks = 0;

	/** GPM: Dimension of bins for finding first theta */
	params->gpm_theta_bin_size_deg = 5.0;
	/** GPM: Area around maximum */
	params->gpm_extend_range_deg = 15.0;
	/** Interval of points to consider (1: all points, 2: every other point, etc */
	params->gpm_interval = 1;

	/** Pose of sensor with respect to robot */
	params->laser[0] = 0.0;
	params->laser[1] = 0.0;
	params->laser[2] = 0.0;

	/** Mark as invalid rays outside of this interval */
	params->min_reading = (double)(min_distance / 1000.0);
	params->max_reading = (double)(max_distance / 1000.0);

	/** If 1, the field 'true_alpha' (or 'alpha') in the first scan is used to compute the incidence beta, and the factor (1/cos^2(beta)) used to weight the correspondence */
	params->use_ml_weights = 0;
	/** If 1, the field 'readings_sigma' in the second scan is used to weight the correspondence by 1/sigma^2 */
	params->use_sigma_weights = 0;

	/** Parameter specific to HSM (unfinished) */
	/** Size of a rho cell */
	params->hsm.linear_cell_size = 0.03;
	/** Size of angular cell (deg) */
	params->hsm.angular_cell_size_deg = 1.0;
	/** Number of angular hypotheses */
	params->hsm.num_angular_hypotheses = 8;
	/** Min distance between directions for cross corr (deg) */
	params->hsm.xc_directions_min_distance_deg = 10.0;
	/** Number of directions for cross corr (deg) */
	params->hsm.xc_ndirections = 3;
	/** Min distance between different angular hypotheses (deg) */
	params->hsm.angular_hyp_min_distance_deg = 10.0;
	/** Number of peaks per direction for linear translation */
	params->hsm.linear_xc_max_npeaks = 5;
	/** Min distance between different peaks in linear correlation */
	params->hsm.linear_xc_peaks_min_distance = 5.0;
}

LDP set_laser_data(long *data, long min_distance, long max_distance, long time_stamp)
{
	int n = 681;
	LDP ld = ld_alloc_new(n);

	/** Minimum and maximum theta (rad) */
	ld->min_theta = -2.0944;
	ld->max_theta =  2.0944;

	int i;
	for(i = 0; i != n; i++) {
		/** Direction of i-th ray with respect to the robot (rad) */
		(ld->theta)[i] = (double)(-2.0944 + 0.00616 * i);
		/** Cluster to which point i belongs */
		(ld->cluster)[i] = -1;
		/** ld->reading[i]: sensor reading(meters)
		 ** ld->valid[i]  : true if this ray is valid */
		if((data[i] < max_distance) && (data[i] > min_distance)) {
			// 对data[i]的数据进行滤波
			data[i] = (long)(data[i] / 10);
			double tmp = (double)data[i];
			(ld->readings)[i] = (double)(tmp / 100.0);
			(ld->valid)[i] = 1;
		}
		else {
			(ld->readings)[i] = NAN;
			(ld->valid)[i] = 0;
		}
	}

	for(i = 0; i != 3; i++) {
		(ld->odometry)[i] = 0.0;
		(ld->estimate)[i] = 0.0;
		(ld->true_pose)[i] = 0.0;
	}

	ld->tv.tv_sec = (int)(time_stamp / 1000.0);
	long sec = (long)(ld->tv.tv_sec) * 1000;
	ld->tv.tv_usec = (int)((time_stamp - sec) * 1000);

	return ld;
}

int set_term(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
    struct termios newtio;

    bzero(&newtio, sizeof(newtio));

    newtio.c_cflag |= CLOCAL | CREAD; // I'm not sure here
    newtio.c_cflag &= ~CSIZE;
    switch(nBits)
    {
    case 7:
        newtio.c_cflag |= CS7;
        break;
    case 8:
        newtio.c_cflag |= CS8;
        break;
    case 9:
        newtio.c_cflag |= CS8;
        break;
    }
    /* Set the Parity Bit */
    switch(nEvent)
    {
    case 'O': // Odd Parity
        newtio.c_cflag |= PARENB;
        newtio.c_cflag |= PARODD;
        newtio.c_iflag |= (INPCK | ISTRIP); // I'm not sure, but it doesn't matters
        break;
    case 'E': // Even Parity
        newtio.c_cflag |= PARENB;
        newtio.c_cflag &= ~PARODD;
        newtio.c_cflag |= (INPCK | ISTRIP); // I'm not sure, but it doesn't matters
        break;
    case 'N': // None Parity
        newtio.c_cflag &= ~PARENB;
        break;
    default:
        newtio.c_cflag &= ~PARENB;
        break;
    }
    /* Set the Baud Rate */
    switch(nSpeed)
    {
    case 2400:
        cfsetispeed(&newtio, B2400);
        cfsetospeed(&newtio, B2400);
        break;
    case 4800:
        cfsetispeed(&newtio, B4800);
        cfsetospeed(&newtio, B4800);
        break;
    case 9600:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    case 115200:
        cfsetispeed(&newtio, B115200);
        cfsetospeed(&newtio, B115200);
        break;
    case 460800:
        cfsetispeed(&newtio, B460800);
        cfsetospeed(&newtio, B460800);
        break;
    default:
        cfsetispeed(&newtio, B9600);
        cfsetospeed(&newtio, B9600);
        break;
    }
    /* Set the Stop Bit */
    if(nStop == 1)
        newtio.c_cflag &= ~CSTOPB;
    else if(nStop == 2)
        newtio.c_cflag |= CSTOPB;

    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 12;

    tcflush(fd, TCIFLUSH);

    if((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
        perror("com set error");
        return -1;
    }

    printf("set done!\n");
    return 0;
}
