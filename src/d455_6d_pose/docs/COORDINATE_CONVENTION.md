# Coordinate and observability contract

All transforms use `parent_T_child`: a reported pose maps object-frame points
into the message header frame. The default header frame is
`camera_color_optical_frame` (`+X` right, `+Y` down, `+Z` forward). When
`output_frame:=base_link` is requested, the node requires a valid hand-eye TF
at the image timestamp and rejects the frame if TF is unavailable.

Energy-unit object frame (pending physical approval): origin at the target-face
centre, `+X` left-to-right, `+Y` bottom-to-top and `+Z` outward from the target
face. Image keypoints remain ordered TL/TR/BR/BL. The PnP solver evaluates both
planar solutions and uses the outward-normal and aligned-depth checks to reject
the mirrored pose. Exact RoboMaster dimensions are placeholders.

An unlabelled cylindrical bottle is axially symmetric. Its centre and principal
axis are observable; rotation around that axis is not. The node publishes a
large covariance and `orientation_complete=false` diagnostic instead of
claiming a unique axial angle.
