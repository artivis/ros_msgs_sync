ros_msgs_sync
===============

Class SyncImplHandler synchronises ros topics (up to 8) of the same kind

Its callback is pure virtual so that you can do whatever you want in a derived class.

Two simple examples are provided :
 - extract_img_sync.launch
     - shows how to inherit
     - extract images from given topics and save them in a directory

 - display_img_sync.launch
     - shows how to inherit
     - display images in OpenCV windows

