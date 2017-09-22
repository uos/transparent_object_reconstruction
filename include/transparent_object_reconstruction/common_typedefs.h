/*
 * Copyright (c) 2015-2017, Sven Albrecht
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 * The views and conclusions contained in the software and documentation are those
 * of the authors and should not be interpreted as representing official policies,
 * either expressed or implied, of the FreeBSD Project.
 */

#ifndef TRANSP_OBJ_RECON_COMMON_DEFS
#define TRANSP_OBJ_RECON_COMMON_DEFS

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

typedef pcl::PointXYZRGBA ColorPoint;
typedef pcl::PointCloud<ColorPoint> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr ConstCloudPtr;

typedef pcl::PointXYZRGBL LabelPoint;
typedef pcl::PointCloud<LabelPoint> LabelCloud;
typedef LabelCloud::Ptr LabelCloudPtr;
typedef LabelCloud::ConstPtr ConstLabelCloudPtr;

static const int ANGLE_RESOLUTION = 360;
static const int OPENING_ANGLE = 10;
static const int MIN_BIN_MARKS = 120;

static const float MEDIAN_FRACTION = 1.0f;


#endif // TRANSP_OBJ_RECON_COMMON_DEFS
