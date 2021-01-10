**********************************************************************************
minidemoKalmanFilter - Minimal Demo. of Kalman Filters (Linear/Extended/Unscented)
**********************************************************************************

Provides
 1. Minimal Kalman Filter classes (Linear, Extended and Unscented).
 2. Interactive demonstration and it's snapshot.
 
This package is very simple, and may suitable for educational use.
 | About 100 executable lines for LKF/EKF/UKF totally.
 | Demo program will show you the essence (assumption and limitation)
 | of Kalman Filter.

Files
-----

``filter.py``
  An implementation of minimal Kalman Filter (LKF/EKF/UKF included).

Demos
-----

Interactive demo
^^^^^^^^^^^^^^^^

Interactive style demo requires ``numpy`` and ``matplotlib``.
Touch sliders to change the parameter of the filter,
and you will find the estimated results updated on your screen.
Some snapshots are included in the package directory (snapshot_*.png).

::

    python demo_ukf_gui.py

![snapshot_01](https://user-images.githubusercontent.com/33892004/104113020-e0d10200-5338-11eb-99c2-56c45457aa2b.png)

![snapshot_02](https://user-images.githubusercontent.com/33892004/104113022-e595b600-5338-11eb-8cea-3e16000a99ce.png)


Batch demo
^^^^^^^^^^

Batch style demo (console version) requires ``numpy``.
This demo estimates the position and velocity of 2-dimensinal 
linear uniform motion, and output results to the console.
You can choose the filter class (LKF,EKF,UKF) by comman line.

LKF, EFK and UKF gives almost same reseults for such a linear 
problem here. Please extend significiant of output to confirm 
the differences.

::

    python demo_ukf.py > out_ukf.txt

Requirements
------------

Uses NumPy and Matplotlib(for interactive demo).

License
-------

Copyright (c) 2018 Kenich SHIRAKAWA

This is licensed under MIT license.
See Licence.txt for more information.

Thanks
------

The basic design of unscented transformation class is based on 
the Sam Burden's work (see https://github.com/sburden/uk ukf.py).

