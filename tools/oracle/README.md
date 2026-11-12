# Oracle

`third_party/aruco_nano.h` is the original ArucoNano (v10), unmodified, MIT licence:
https://github.com/rmsalinas/aruco_nano/blob/c99989bcab795ffa65d9e12df9de15f27677a835/aruco_nano.h

It is used only by `ppmt_fiducial_oracle` to produce reference output and is never linked
into the library. It needs OpenCV >= 4.7; `compat/` makes it build on older OpenCV (4.5.4)
and is the only place that difference is handled.

`make oracle` runs it on `data/sample/synthetic/` (dictionary APRILTAG_36h11, otherwise
default parameters) and writes `data/oracle/synthetic/<name>.yaml`.
