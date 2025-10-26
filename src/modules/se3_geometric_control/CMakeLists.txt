px4_add_module(
    MODULE modules__se3_geometric_control
    MAIN se3_geometric_control
    SRCS se3_geometric_control.cpp
    LINK_LIBRARIES
        Pinocchio::pinocchio  # 关键：链接Pinocchio库
)
