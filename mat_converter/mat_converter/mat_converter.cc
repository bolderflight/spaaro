/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2021 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#include <stdio.h>
#include <google/protobuf/message.h>
#include <iostream>
#include "datalog.pb.h"
#include "framing/framing.h"
#include "mat_v4/mat_v4.h"
#include "Eigen/Core"
#include "Eigen/Dense"

int main(int argc, char** argv) {
  /* Verify version of protobuf */
  GOOGLE_PROTOBUF_VERIFY_VERSION;
  /* Grab the input filename */
  if (argc != 2) {
    std::cerr << "Usage:  " << argv[0] << " <FLIGHT DATA FILE>" << std::endl;
    return -1;
  }
  /* Input file name */
  std::string input_file_name(argv[1]);
  /* Check input file extension */
  std::string bfs_ext = ".bfs";
  std::cout << "Parsing file " << input_file_name << "...";
  if (input_file_name.compare(input_file_name.length() - bfs_ext.length(), bfs_ext.length(), bfs_ext) != 0) {
    std::cerr << "ERROR: Input file must be a BFS log file, which has a .bfs extension." << std::endl;
    return -1;
  }
  /* Try to read the flight data */
  FILE *input = fopen(input_file_name.c_str(), "rb");
  if (!input) {
    std::cerr << "ERROR: Unable to open input file, maybe the path is incorrect." << std::endl;
    return -1;
  }
  /* Create the output file */
  std::string mat_ext = ".mat";
  std::string output_file_name = input_file_name;
  output_file_name.replace(output_file_name.length() - bfs_ext.length(), bfs_ext.length(), mat_ext);
  FILE *output = fopen(output_file_name.c_str(), "wb");
  if (!output) {
    std::cerr << "ERROR: Unable to open output file." << std::endl;
    return -1;
  }
  /* The message type */
  DatalogMessage datalog;
  /* Read file in chunks */
  static constexpr int CHUNK_SIZE = 1024;
  uint8_t buffer[CHUNK_SIZE];
  std::size_t bytes_read = 0;
  /* Framing */
  bfs::Decoder<CHUNK_SIZE> temp_decoder;
  /* Iterate through the file once to get the length to allow us to pre-allocate arrays */
  std::size_t num_packets = 0;
  while ((bytes_read = fread(buffer, 1, sizeof(buffer), input)) > 0) {
    for (std::size_t i = 0; i < bytes_read; i++) {
      if (temp_decoder.Found(buffer[i])) {
        if (datalog.ParseFromArray(temp_decoder.Data(), temp_decoder.Size())) {
          num_packets++;
        }
      }
    }
  }
  rewind(input);
  /* Get the datalog descriptors */
  const google::protobuf::Descriptor* descriptor = datalog.GetDescriptor();
  /* 
  * Datalog reflection, this should be done on each packet, but
  * we're assuming that the packets don't change.
  */
  const google::protobuf::Reflection* reflection = datalog.GetReflection();
  /* Number of fields */
  std::size_t field_count = descriptor->field_count();
  /* Iterate through fields */
  for (std::size_t field = 0; field < field_count; field++) {
    /* Get the field name */
    std::string field_name = descriptor->field(field)->name();
    /* Get the data type */
    google::protobuf::FieldDescriptor::CppType cpp_type = descriptor->field(field)->cpp_type();
    /* Get the label */
    google::protobuf::FieldDescriptor::Label label = descriptor->field(field)->label();
    /* Data columns */
    std::size_t cols;
    /* Get the size if repeated */
    if (label == google::protobuf::FieldDescriptor::LABEL_REPEATED) {
      cols = reflection->FieldSize(datalog, descriptor->field(field));
    } else {
      cols = 1;
    }
    /* Switch based on type */
    switch (cpp_type) {
      case google::protobuf::FieldDescriptor::CPPTYPE_INT32: {
        /* Create a int32 container */
        Eigen::Matrix<int32_t, Eigen::Dynamic, Eigen::Dynamic> val;
        val.resize(num_packets, cols);
        /* Read the file */
        bfs::Decoder<CHUNK_SIZE> decoder;
        std::size_t packet = 0;
        while ((bytes_read = fread(buffer, 1, sizeof(buffer), input)) > 0) {
          for (std::size_t i = 0; i < bytes_read; i++) {
            if (decoder.Found(buffer[i])) {
              if (datalog.ParseFromArray(decoder.Data(), decoder.Size())) {
                if (label == google::protobuf::FieldDescriptor::LABEL_REPEATED) {
                  for (std::size_t j = 0; j < cols; j++) {
                    val(packet, j) = reflection->GetRepeatedInt32(datalog, descriptor->field(field), j);
                  }
                  packet++;
                } else {
                  val(packet++, 0) = reflection->GetInt32(datalog, descriptor->field(field));
                }
              }
            }
          }
        }
        rewind(input);
        /* Write MATLAB output */
        bfs::MatWrite(field_name, val, output);
        break;
      }
      case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE: {
        /* Create a double container */
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> val;
        val.resize(num_packets, cols);
        /* Read the file */
        bfs::Decoder<CHUNK_SIZE> decoder;
        std::size_t packet = 0;
        while ((bytes_read = fread(buffer, 1, sizeof(buffer), input)) > 0) {
          for (std::size_t i = 0; i < bytes_read; i++) {
            if (decoder.Found(buffer[i])) {
              if (datalog.ParseFromArray(decoder.Data(), decoder.Size())) {
                if (label == google::protobuf::FieldDescriptor::LABEL_REPEATED) {
                  for (std::size_t j = 0; j < cols; j++) {
                    val(packet, j) = reflection->GetRepeatedDouble(datalog, descriptor->field(field), j);
                  }
                  packet++;
                } else {
                  val(packet++, 0) = reflection->GetDouble(datalog, descriptor->field(field));
                }
              }
            }
          }
        }
        rewind(input);
        /* Write MATLAB output */
        bfs::MatWrite(field_name, val, output);
        break;
      }
      case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT: {
        /* Create a float container */
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> val;
        val.resize(num_packets, cols);
        /* Read the file */
        bfs::Decoder<CHUNK_SIZE> decoder;
        std::size_t packet = 0;
        while ((bytes_read = fread(buffer, 1, sizeof(buffer), input)) > 0) {
          for (std::size_t i = 0; i < bytes_read; i++) {
            if (decoder.Found(buffer[i])) {
              if (datalog.ParseFromArray(decoder.Data(), decoder.Size())) {
                if (label == google::protobuf::FieldDescriptor::LABEL_REPEATED) {
                  for (std::size_t j = 0; j < cols; j++) {
                    val(packet, j) = reflection->GetRepeatedFloat(datalog, descriptor->field(field), j);
                  }
                  packet++;
                } else {
                  val(packet++, 0) = reflection->GetFloat(datalog, descriptor->field(field));
                }
              }
            }
          }
        }
        rewind(input);
        /* Write MATLAB output */
        bfs::MatWrite(field_name, val, output);
        break;
      }
      case google::protobuf::FieldDescriptor::CPPTYPE_BOOL: {
        /* Create a uint8 container */
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic> val;
        val.resize(num_packets, cols);
        /* Read the file */
        bfs::Decoder<CHUNK_SIZE> decoder;
        std::size_t packet = 0;
        while ((bytes_read = fread(buffer, 1, sizeof(buffer), input)) > 0) {
          for (std::size_t i = 0; i < bytes_read; i++) {
            if (decoder.Found(buffer[i])) {
              if (datalog.ParseFromArray(decoder.Data(), decoder.Size())) {
                if (label == google::protobuf::FieldDescriptor::LABEL_REPEATED) {
                  for (std::size_t j = 0; j < cols; j++) {
                    val(packet, j) = reflection->GetRepeatedBool(datalog, descriptor->field(field), j);
                  }
                  packet++;
                } else {
                  val(packet++, 0) = reflection->GetBool(datalog, descriptor->field(field));
                }
              }
            }
          }
        }
        rewind(input);
        /* Write MATLAB output */
        bfs::MatWrite(field_name, val, output);
        break;        
      }
      default: {
        // std::cout << cpp_type << std::endl;
        // std::cerr << "ERROR: Unsupported data type." << std::endl;
        // return -1;
      }
    }
  }
  /* Print out closing info */
  std::cout << "done." << std::endl;
  std::cout << "Wrote " << field_count << " fields and " << num_packets << " data packets." << std::endl;
  std::cout << "Saved as " << output_file_name << std::endl;
  return 0;
}

