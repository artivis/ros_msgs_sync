ros_img_sync
===============

Class SyncImageHandler synchronises image topics (up to 8).

Its callback is pure virtual so that you can do whatever you want in a derived class.

Two simple examples are provided :
  extract_img_sync.launch shows how to inherit and extract images from given topics and save them in a directory.
  display_img_sync.launch           //         and display images in OpenCV windows.

