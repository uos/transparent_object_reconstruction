transparent_object_reconstruction
===========================

This package contains methods for detection of potential locations and reconstruction of transparent objects in organized point clouds provided by kinect / xtion sensors.
The challenging part here is that transparent (or highly reflective) objects do not provide range measurements from their surface, but usually don't provide sensor information, i.e., the associated depth value is a nan-value.
Note that this package is work in progress and there are no guarantees that the current code (remotely) does what it is actually supposed to do.

* Holes.msg describes a new type of message to pass along the convex hull of (at least) one hole detected in a point cloud.
Each convex hull is described as a point cloud, containing the (ordered) points that compose the convex hull.
* HoleDetector provides an ecto cell that expects as inputs an organized point cloud, the model coefficients of a planar surface (e.g. a tabletop) in said point cloud and the point indices of the convex hull of the detected planar region.
The organized point cloud is searched for 'holes', i.e., clusters of points with nan-values as measurements.
Clusters that lie at least partially inside the convex hull of the planar surface are candidate locations for transparent objects.
As such the convex hull of their borders are computed (respectively the convex hull of the border points projected into the planar surface in case of clusters that are only partially inside the convex hull of the planar surface) and published as a message of type 'Holes.msg'
* HoleIntersector in turn expects in its callback the arrival of 'Holes.msg'.
For each contained convex hull a point sample of the enclosed area is created and from this a frustum is created with the camera position at its tip.
Such a frustum effectively represents the upper limit of the volume of an object that might have caused the cluster of nan-values in the organized point cloud.
Provided with further 'Holes.msg', that were obtained from point clouds of the same scene, but from a different viewpoint, the intersection of these 'occlusion frusta' can approximate the actual shape of the transparent object, provided the point clouds are correctly registered with respect to each other.
