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

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/filesystem.hpp>

#include <pcl/console/parse.h>

#include <iostream>
#include <fstream>

namespace fs = boost::filesystem;

void
usage (int arg, char **argv)
{
  std::cout << "usage:\nrosrun transparent_object_reconstruction pose2quatTrans"
    <<" -f pose_file" << std::endl;
}

int
main (int argc, char **argv)
{
  if (!(pcl::console::find_argument (argc, argv, "-f") > 0))
  {
    usage (argc, argv);
    return EXIT_FAILURE;
  }

  std::string pose_filename;

  pcl::console::parse_argument (argc, argv, "-f", pose_filename);

  bool write_pose = false;
  std::string out_filename;
  if (pcl::console::find_argument (argc, argv, "-w") > 0)
  {
    fs::path tmp_path (pose_filename);
    out_filename = tmp_path.stem ().string () + ".transQuat";
    write_pose = true;
  }

  std::ifstream pose_file (pose_filename.c_str ());

  Eigen::Matrix4f p_mat;
  float value;
  for (size_t i = 0; i < 4; ++i)
  {
    for (size_t j = 0; j < 4; ++j)
    {
      pose_file >> value;
      p_mat (i, j) = value;
    }
  }
  if (!pose_file.good ())
  {
    std::cerr << "specified file '" << pose_filename
      << "' indicates error while reading 4x4 pose matrix"
      << "\nExiting." << std::endl;
    return EXIT_FAILURE;
  }

  pose_file.close ();

  Eigen::Affine3f pose (p_mat);

  Eigen::Quaternion<float> q (pose.rotation ());
  Eigen::Vector3f t = pose.translation ();

  std::cout << t[0] << " " << t[1] << " " << t[2] << " ";
  std::cout << q.x () << " " << q.y () << " " << q.z () << " " << q.w () << std::endl;

  if (write_pose)
  {
    std::ofstream out_file (out_filename.c_str ());
    out_file << t[0] << " " << t[1] << " " << t[2] << std::endl;
    out_file << q.x () << " " << q.y () << " " << q.z () << " " << q.w () << std::endl;
    out_file.flush ();
    out_file.close ();
  }

  return EXIT_SUCCESS;
}
