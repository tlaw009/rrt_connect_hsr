# RRT_connect_hsr
RRT implementation for manipulation with HSR utilizing kinematic solver from drake

Current: trimmed down urdf, identified ```kInfeasibleConstraints``` issue to be inherent rotational offset between base_footprint and hand_palm_link being unaccounted for when pose passed to solver

Repo log:

repo name changed to RRT_connect_hsr

repo name changed to rrt_connect_hsr to conform to rospkg naming convention