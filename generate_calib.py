import numpy as np
import rosbag
from rospy.rostime import Duration, Time
from tactile_msgs.msg import TactileState
import argparse
from collections import OrderedDict
import csv
import yaml
import time

#from sklearn import linear_model
import pwlf


#from scipy.interpolate import UnivariateSpline

#from sklearn.tree import DecisionTreeRegressor
#from sklearn.linear_model import LinearRegression



import matplotlib.pyplot as plt

#Calibration Procedure:
#: rosbag record  /TactileGlove -o <outputfilename>

#Glove: P2 with Teensy3.2, ADC: AD7490
#CalibTool Channel: TactileGlove/sensors[0]/values[17]


REF_CALIB_RATIO = -0.0154
REF_CALIB_OFFSET = 53.793
DEFAULT_INPUT_RESOL = 12 # in bits
CHANGE_DETECT_THRESH = 4 # 5 unit of velocity positive or negative
DEFAULT_SEGMENTS = 5


def smooth(y, box_pts):
    box = np.ones(box_pts)/box_pts
    y_smooth = np.convolve(y, box, mode='same')
    return y_smooth

def find_inflection_index(x):
    direction = 1 # check for increasing
    increasing_idx = []
    decreasing_idx = []
    prev_val = 0
    for i, val in enumerate(vel):
        if direction == 1: # checking for increasing
            if val > CHANGE_DETECT_THRESH: # if above threshold consider increasing
                direction = 2 # switch to increasing
                increasing_idx.append(i)
                continue
        if direction == 2: # increasing
            if val > 0:
                increasing_idx.append(i)
                continue
            else:
                direction = -1 # switch to check for decreasing
                continue
        if direction == -1:
            if val < -CHANGE_DETECT_THRESH: # if below threshold consider decreasing
                direction = -2 # switch to decreasing
                decreasing_idx.append(i)
                continue
        if direction == -2: # decreasing
            if val < 0:
                decreasing_idx.append(i)
                continue
            else:
                direction = 1 # switch to check for increasing
                continue
    return [np.array(increasing_idx), np.array(decreasing_idx)]


if __name__ == "__main__":
    # execute only if run as a script
    parser = argparse.ArgumentParser()

    # args : 1 bagfilename, 2 topic, sensor number, calib number
    parser.add_argument("bagfilename", type=str,
                      help="bag filename to open")
    parser.add_argument("topic", type=str,
                      help="topic to process")
    parser.add_argument("data_channel", type=int,
                      help="index of the data channel to calibrate")
    parser.add_argument("ref_channel", type=int,
                      help="index of the ref channel to calibrate against")
    parser.add_argument("ref_tare_val", type=float,
                      help="reference tare value (given in newton)")
    parser.add_argument("--ref_ratio", type=float, default=REF_CALIB_RATIO,
                      help="reference ratio (indicated on the tool)")
    parser.add_argument("--ref_offset", type=float, default=REF_CALIB_OFFSET,
                      help="reference offset (indicated on the tool)")
    parser.add_argument("--plot",  action="store_true", help="show the plots")
    parser.add_argument("--input_resolution", type=int, default=DEFAULT_INPUT_RESOL,
                      help="input resolution in bits")
    parser.add_argument("--segments", type=int, default=DEFAULT_SEGMENTS,
                      help="number of segments for piece-wise-linear")
    args = parser.parse_args()


    input_range_max = 2**args.input_resolution
    # read the bag file
    bag = rosbag.Bag(args.bagfilename)
    ref_raw_vec = []
    raw_vec = []
    # TODO Guillaume : also handle the sensor name in case there are more than one sensor (like on iObject+)
    sensor_name = None
    print "looking for ", args.topic
    for topic, msg, t in bag.read_messages(topics=[args.topic]):
        # if there is data
        if len(msg.sensors):
            if sensor_name is None:
                sensor_name = msg.sensors[0].name
            # validate index are in the range
            if (args.ref_channel < len(msg.sensors[0].values) and args.data_channel < len(msg.sensors[0].values)):
                if msg.sensors[0].values[args.data_channel] < input_range_max:
                    # extract the dedicated channels for raw and reference_raw
                    # 2.d    crop saved values to pair of channels [CalibTool;determined taxe]: /TactileGlove/sensors[0]/values[17] and /TactileGlove/sensors[0]/values[determined taxel]
                    ref_raw_vec.append(msg.sensors[0].values[args.ref_channel])
                    raw_vec.append(msg.sensors[0].values[args.data_channel])
                else: # input range badly chosen
                    print "Data ", msg.sensors[0].values[args.data_channel], " is higher than input resolution ", input_range_max
                    bag.close()
                    exit(-1)
            # else drop the data of this message
    bag.close()
    if (len (raw_vec)==0 or len(ref_raw_vec) ==0):
        print "no data retrieved, check topic name"
        exit(-1)

    # process the data
    # 3. Offline Lookuptable generation
    # convert the data
    ref_raw = np.array(ref_raw_vec)
    raw = np.array(raw_vec)

    # compute ref in newton 3.b    Calculate Ground Truth in Newton GTN: with Model GTN = -0,0154 * CtRAW+ 53,793    where   CrRAW = Calib-tool-RAW [12Bit] = "field.sensors0.values17" out of recorded rosbag file.
    ref_newton = args.ref_ratio * ref_raw + args.ref_offset
     # tare 3.c    Tare with each: GTtN = GTN - Mean(TareValues)
    ref_newton_tare = ref_newton - args.ref_tare_val

    # find the sections in which pressure increases/decreases
    # TODO Guillaume : also look at the range of ref data to not get a decreasing force at the end of the increasing raw data
    ## smooth the data to avoid derivate sign change on noise
    smooth_raw = smooth(raw,19)
    ## compute derivative of the input data
    vel = np.gradient(smooth_raw)
    ## find inflections
    [inc_idx, dec_idx]=find_inflection_index(vel);

    # 3.a View recorded data in graphplot in order to validate correctness (no spurious wrong data or high noise)
    if args.plot:
        plt.plot(range(len(smooth_raw)),smooth_raw, 'g-', lw=1) # smooth raw values
        plt.plot(range(len(ref_newton_tare)),ref_newton_tare*-100.0, 'm-', lw=1) # smooth ref values, on negative side to avoid cluttering
        plt.plot(inc_idx, smooth_raw[inc_idx], 'rx', lw=2) # increasing values
        plt.plot(dec_idx, smooth_raw[dec_idx], 'bx', lw=2) # decreasing values
        plt.plot(range(len(vel)), vel*50, 'k-', lw=1) # velocity scaled up
        plt.show()

    # 3.d0    Database funktion: With each lookuptable entrance "Lookup[0-4095]"  (in excel it is  AVERAGEIF function)
    missing_inc_input_values = [] # list of increasing input values that do not appear in the recorded data.
    missing_dec_input_values = [] # list of decreasing input values that do not appear in the recorded data.
    lookup_inc = OrderedDict()
    lookup_dec = OrderedDict()
    for val in range(input_range_max):

        # Get the index of increasing elements with value val   3.d1        Seek Lookup(n=0) within RAW-Taxel-values
        #print "inc_idx ", inc_idx
        idx_of_val_inc = np.where(raw[inc_idx] == val)
        #print "idx_of_val_inc ", idx_of_val_inc
        #print "inc_idx[idx_of_val_inc] ",inc_idx[idx_of_val_inc]
        #print "ref_newton_tare ", ref_newton_tare[inc_idx[idx_of_val_inc]]

        if len(idx_of_val_inc[0]) > 0:
            # TODO: depending how many index were found, process or not
            # store mean val 3.d2-d5
            lookup_inc[val] = round(np.mean(ref_newton_tare[inc_idx[idx_of_val_inc]]),3);
        else: # no element for that value, store it in an index
            missing_inc_input_values.append(val)

        # Get the index of decreasing elements with value val   3.d1        Seek Lookup(n=0) within RAW-Taxel-values
        idx_of_val_dec = np.where(raw[dec_idx] == val)
        if len(idx_of_val_dec[0]) > 0:
            # TODO: depending how many index were found, process or not
            # store mean val 3.d2-d5
            lookup_dec[val] = round(np.mean(ref_newton_tare[dec_idx[idx_of_val_dec]]),3);
        else: # no element for that value, store it in an index
            missing_dec_input_values.append(val)

    lists_lookup_inc = lookup_inc.items() # sorted by key, return a list of tuples
    x_inc, y_inc = zip(*lists_lookup_inc) # unpack a list of pairs into two tuples
    
    lists_lookup_dec = lookup_dec.items() # sorted by key, return a list of tuples
    x_dec, y_dec = zip(*lists_lookup_dec) # unpack a list of pairs into two tuples
    
    # e    Visualize point diagram with [0-4095;LGTtN] including all taxel data points in X axis =[0-4095] over Y-axis =[fmin-fmax Newton]
    if args.plot:
        plt.plot(x_inc, y_inc, 'mx')
        plt.plot(x_inc, smooth(y_inc,100), 'r')
        plt.plot(x_dec, y_dec, 'c+')
        plt.plot(x_dec, smooth(y_dec,100), 'b')
        plt.show()

    # process only increasing
    xs = np.array(x_inc)
    ys = np.array(y_inc)
    #ys = smooth(y_inc,100)

    # solution 0 not tested
    #clf = linear_model.LinearRegression()
    #clf.fit([[getattr(t, 'x%d' % i) for i in range(1, 8)] for t in texts],
    #    [t.y for t in texts])

    # solution with pwlf 1  https://pypi.org/project/pwlf/   # not available in package manager
    # Installing collected packages: numpy, scipy, pyDOE, pwlf
    # Successfully installed numpy-1.16.6 pwlf-2.0.3 pyDOE-0.3.8 scipy-1.2.3

    pwlf_result = pwlf.PiecewiseLinFit(xs, ys)
    # request a fit with n points
    pwlf_breaks = pwlf_result.fit(args.segments)
    
    #print(breaks)
    if args.plot:
        x_hat = np.linspace(xs.min(), xs.max(), 100)
        y_hat = pwlf_result.predict(x_hat)

        plt.figure()
        plt.plot(xs, ys, 'o')
        plt.plot(x_hat, y_hat, '-')
        plt.show()

    # Solution with splines 2
    #spl = UnivariateSpline(x, y, k=1, s=0.5)
    #xs = np.linspace(x.min(), x.max(), 100)
    #fig, ax = plt.subplots()
    #ax.scatter(x, y, color="red", s=20, zorder=20)
    #ax.plot(xs, spl(xs), linestyle="--", linewidth=1, color="blue", zorder=10)
    #ax.grid(color="grey", linestyle="--", linewidth=.5, alpha=.5)
    #ax.set_ylabel("Y")
    #ax.set_xlabel("X")
    #plt.show()

    # solution with sklearn 3  sudo apt-get install python-sklearn

    # segmented linear regression parameters
    # n_seg = 10

    ##fig, (ax0, ax1) = plt.subplots(1, 2)
    #fig, ax0 = plt.subplots(1, 1)

    # dys = np.gradient(ys, xs)
    # rgr = DecisionTreeRegressor(max_leaf_nodes=n_seg)
    # rgr.fit(xs.reshape(-1, 1), dys.reshape(-1, 1))
    # dys_dt = rgr.predict(xs.reshape(-1, 1)).flatten()

    # ys_sl = np.ones(len(xs)) * np.nan
    # for y in np.unique(dys_dt):
        # msk = dys_dt == y
        # lin_reg = LinearRegression()
        # lin_reg.fit(xs[msk].reshape(-1, 1), ys[msk].reshape(-1, 1))
        # ys_sl[msk] = lin_reg.predict(xs[msk].reshape(-1, 1)).flatten()
        # ax0.plot([xs[msk][0], xs[msk][-1]],
                 # [ys_sl[msk][0], ys_sl[msk][-1]],
                 # color='r', zorder=1)

    # ax0.set_title('values')
    # ax0.scatter(xs, ys, label='data', color='k')
    # ax0.scatter(xs, ys_sl, s=3**2, label='seg lin reg', color='g', zorder=5)
    # ax0.legend()

    # #ax1.set_title('slope')
    # #ax1.scatter(xs, dys, label='data')
    # #ax1.scatter(xs, dys_dt, label='DecisionTree', s=2**2)
    # #ax1.legend()
    # plt.show()

    # with open('lookup_inc.csv', 'w') as csvfile:
        # csvwriter = csv.writer(csvfile, delimiter=',')
        # for key in lookup_inc:
            # csvwriter.writerow([key, lookup_inc[key]])
    # with open('lookup_dec.csv', 'w') as csvfile:
        # csvwriter = csv.writer(csvfile, delimiter=',')
        # for key in lookup_dec:
            # csvwriter.writerow([key, lookup_dec[key]])

    # extract the lookup_values
    mapping_dict = OrderedDict()
    for b in pwlf_breaks:
        mapping_dict[round(float(b),2)] =  round(float(pwlf_result.predict(round(float(b),2))[0]),3)
    #print mapping_list
    with open('lookup.csv', 'w') as csvfile:
        csvwriter = csv.writer(csvfile, delimiter=',')
        for x in mapping_dict:
            csvwriter.writerow([x, mapping_dict[x]])
            
    # 6. Save
    # a    Save Lookuptable and-or Model in TaxelCalibrationMapping file.
    #yaml.add_representer(OrderedDict, lambda dumper, data: dumper.represent_mapping('tag:yaml.org,2002:map', data.items()))
    #yaml.add_representer(tuple, lambda dumper, data: dumper.represent_sequence('tag:yaml.org,2002:seq', data))
    represent_dict_order = lambda self, data:  self.represent_mapping('tag:yaml.org,2002:map', data.items())
    yaml.add_representer(OrderedDict, represent_dict_order)  

    calib = {"calib": [{'sensor_name': sensor_name, 'type': 'PWL', 'idx_range': [args.data_channel], "values": mapping_dict}]}
    with open('mapping.yaml', 'w') as f:
        data = yaml.dump(calib, f)  #sort_keys=False in python3 only     

    #time.sleep(10)



