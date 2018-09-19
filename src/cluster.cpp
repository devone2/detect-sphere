#include <iostream>
#include <fstream>

#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"

#include "common.h"

namespace fs = boost::filesystem;



int main(int argc, char** argv) {

  if(argc != 3) {
    std::cout << "Provide input file as an only argument";
    return 1;
  }

  std::string dir = argv[1];
  fs::path out_dir = fs::path(argv[2]);
  fs::path out_results = fs::path(out_dir);
    out_results /= "results.csv";
  fs::directory_iterator end_iter;

  std::ofstream result_file;
  result_file.open(out_results.string());
  result_file << "file;found;points;x;y;z;r\n";

  for ( fs::directory_iterator dir_itr( dir );
        dir_itr != end_iter;
        ++dir_itr )
    {
      if ( fs::is_regular_file( dir_itr->status() ))
            {
              fs::path infilename = dir_itr->path().filename();
              fs::path outfile = fs::path(out_dir) /= infilename;
              std::cout << dir_itr->path().filename() << " to " << outfile << "\n";
              SegResult result;
              find(dir_itr->path().string(),outfile.string(), result);
              result_file << infilename << ";" ;
                if(result.is_empty()) {
                  result_file << "N" << std::endl;
                } else {
                  auto c = result.coef->values;
                  result_file << "Y;" <<  result.size() << ";" << c[0] << ";" << c[1] << ";" << c[2] << ";" << c[3] << std::endl;
                }

            }
    }

   result_file.close();
}

