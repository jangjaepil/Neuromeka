.. _linear_system_solvers_setting :

Linear Systems Solvers
-----------------------
The settings parameter :code:`linsys_solver` defines the solver for the linear system.
In C it corresponds to an integer :code:`OSQPInt` (see :ref:`c_cpp_data_types`) and in the other high level languages to a string.


+-----------------+-------------------+--------------------------------+---------------+
| Solver          | String option     | C     Constant                 | Integer value |
+=================+===================+================================+===============+
| QDLDL           | "qdldl"           | :code:`QDLDL_SOLVER`           | :code:`0`     |
+-----------------+-------------------+--------------------------------+---------------+
| MKL Pardiso     | "mkl pardiso"     | :code:`MKL_PARDISO_SOLVER`     | :code:`1`     |
+-----------------+-------------------+--------------------------------+---------------+
| CUDA PCG        | "cuda pcg"        | :code:`CUDA_PCG_SOLVER`        | :code:`2`     |
+-----------------+-------------------+--------------------------------+---------------+



To add new linear system solvers see :ref:`interfacing_new_linear_system_solvers`.



