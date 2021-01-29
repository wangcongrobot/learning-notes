# rosbag & matlab


## extract a time series
The Mathworks has some good instructions on the basics:

http://www.mathworks.com/help/robotics/examples/work-with-rosbag-logfiles.html

http://www.mathworks.com/help/robotics/ug/ros-log-files-rosbags.html

However, the tools make use of some not-so-simple MATLAB data structures - MATLAB tables and timeseries objects in particular.

Here is an example that shows some tricks for determining how to extract a time series
```matlab
bag = rosbag('../data/myturtle.bag')
% Get some information about what topics are in the bag file
% Returns a "table" object
atopics = bag.AvailableTopics
summary(atopics)
% The ROS topics are in the RowNames - list them.
atopics.Properties.RowNames
% We want to plot the pose, so lets look at the
% definition of that message to see what fields are available
msg = atopics('/turtle1/pose',:)
msg.MessageDefinition{1} % This should display the fields of the message

% Get all the messages on a single topic
bagselect = select(bag,'Topic','turtle1/pose');
msgs = readMessages(bagselect);
m=msgs{1} % to get some idea of the message structure

ts = timeseries(bagselect,'X','Y','Theta','LinearVelocity','AngularVelocity');
```

## rosbag rostopic select



We can see that there are a fair number of topics.   We are really just interested in a few, so we can extract those topics (just three topics in this example), saving a subset of the data to a new file called nick_drivebox1_2017-03-27-16-10-19_selected.bag
```bash
rosbag filter nick_drivebox1_2017-03-27-16-10-19.bag nick_drivebox1_2017-03-27-16-10-19_selected.bag 'topic == "/geonav_odom" or topic == "/cmd_drive" or topic == "/cmd_course"'
```

## code

https://github.com/rwightman/udacity-driving-reader/blob/master/script/bagdump.py

https://github.com/uzh-rpg/rpg_feature_tracking_analysis/blob/master/big_pun/bag2dataframe.py

https://github.com/intel/driving-data-collection-reference-kit/blob/master/data_extraction/extract_data.py

https://github.com/OpenAgricultureFoundation/openag_cv/blob/master/utils/Bag2CSV.py

https://github.com/tsaoyu/rosbag_pandas/blob/master/scripts/bag_print

