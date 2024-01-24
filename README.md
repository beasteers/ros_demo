# ROS Demo

```bash
docker-compose up -d
```

## Summary

Core ROS services:
- internal ROS communication:
  - fastdds: Discovery server so ROS nodes can talk to each other
- external ROS communication:
  - foxglove_bridge: So non-ROS services can communicate using websockets
  - rosbridge_server: Non-foxglove implementation of ROS websockets
- ros_record: A ROS node for data recording
- UI
  - foxglove_studio: Foxglove UI
  - minio: S3 storage
  - directus

Example services:
 - publish: publishes video
 - listen: displays video

TODO:
 - evaluate FPS/latency
   - foxglove vs rospy vs rosbridge
 - recorder node
 - foxglove UI configurations
   - [record button](https://docs.foxglove.dev/docs/visualization/panels/service-call)
