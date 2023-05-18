# Topic Buffer (ROS1)

Buffers incoming messages and adds a delay before re-publishing. Buffered messages can be deleted using the service. This package provides the application as a nodelet to enable memory efficient processing.



## Demo

Run:

```bash
$ roslaunch topic_buffer demo.launch
```

Then in different terminals run:

- `rostopic echo /chatter`
  - This is the published data.
- `rostopic echo /delayed_chatter`
  - This is the data delayed for 2 seconds.
- `rosrun rqt_service_caller rqt_service_caller`
  - Try calling `/topic_buffer_node/clear_buffer`

## ROS

Topics:

- `~input`: Input topic.
- `~output`: Output topic.

Services:

- `~clear_buffer`: Buffer clear service. (`std_srvs/Empty`)

Parameters

- `~delay`: Amount of delay in seconds. (1.0 by default)