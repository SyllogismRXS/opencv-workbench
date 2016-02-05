#ifndef PLUGINMANAGER_H_
#define PLUGINMANAGER_H_
/// ---------------------------------------------------------------------------
/// @file PluginManager.h
/// @author Kevin DeMarco <kevin.demarco@gmail.com>
///
/// Time-stamp: <2016-02-05 09:30:57 syllogismrxs>
///
/// @version 1.0
/// Created: 04 Feb 2015
///
/// ---------------------------------------------------------------------------
/// @section LICENSE
///
/// The MIT License (MIT)
/// Copyright (c) 2012 Kevin DeMarco
///
/// Permission is hereby granted, free of charge, to any person obtaining a
/// copy of this software and associated documentation files (the "Software"),
/// to deal in the Software without restriction, including without limitation
/// the rights to use, copy, modify, merge, publish, distribute, sublicense,
/// and/or sell copies of the Software, and to permit persons to whom the
/// Software is furnished to do so, subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
/// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
/// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
/// DEALINGS IN THE SOFTWARE.
/// ---------------------------------------------------------------------------
/// @section DESCRIPTION
///
/// The PluginManager class ...
///
/// ---------------------------------------------------------------------------
#include <iostream>
#include <dlfcn.h>
#include <stdio.h>
#include <cstdlib>
#include <unistd.h>
#include <stdlib.h>
#include <dlfcn.h>
#include <list>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/foreach.hpp>
#include <boost/tokenizer.hpp>

using std::cout;
using std::endl;

using namespace std;

namespace fs = ::boost::filesystem;

template <typename T, typename M>
     class PluginManager {
public:
     PluginManager() {}

     typename std::map<std::string, M *, std::less<std::string> > factory;

     static const int NON_AVIA_LIBRARY = -1;
     static const int FAILED_TO_OPEN_LIBRARY = -2;

     // Give an environment variable and a file extension:
     // Find the absolute path to all files in environment variable paths
     std::vector<std::string> find_files(std::string env_var, const std::string &ext)
     {
          std::vector<std::string> paths;

          // Get the AVIA_PLUGIN_PATH environment variable
          const char* env_p = std::getenv(env_var.c_str());
          if (env_p == NULL) {
               cout << env_var << " environment variable not set" << endl;
               return paths;
          }

          std::string env_path = std::string(env_p);
          //cout << env_var << ": " << env_path << endl;

          // Tokenize the path and loop through each directory
          boost::char_separator<char> sep(":");
          boost::tokenizer< boost::char_separator<char> > tokens(env_path, sep);
          BOOST_FOREACH (const string& t, tokens) {
               // Search for all files in the current directory with
               // the extension
               fs::path root = t;
               if(fs::exists(root) && fs::is_directory(root)) {
                    //cout << "Searching plugin path: " << t << endl;

                    fs::recursive_directory_iterator it(root);
                    fs::recursive_directory_iterator endit;

                    while(it != endit) {
                         if(fs::is_regular_file(*it) && it->path().extension() == ext) {
                              std::string full_path = fs::absolute(it->path()).string();
                              paths.push_back(full_path);
                         }
                         ++it;
                    }
               } else {
                    cout << "Plugin path doesn't exist: " << t << endl;
               }
          }

          return paths;
     }

     int search_for_plugins(std::string env_var)
     {
          std::vector<std::string> files;
          std::vector<std::string>::iterator it;

          // Get all files within environment variable that are
          // linux shared libraries
          files = find_files(env_var, ".so");
          for(it = files.begin(); it < files.end(); it++) {
               // Load each library found
               int result = this->load_library(*it);
               if (result  == FAILED_TO_OPEN_LIBRARY) {
                    //cout << "Can't load lib: " << *it << endl;
               }
          }

          this->finalize_libraries();

          return 0;
     }

     int load_library(std::string lib_name)
     {
          void *dlib;
          unsigned int size = factory.size();

          dlib = dlopen(lib_name.c_str(), RTLD_NOW);
          if(dlib == NULL){
               //std::cerr << dlerror() << endl;
               //cout << dlerror() << endl;
               dlerror();
               return FAILED_TO_OPEN_LIBRARY;
          }
          
          if (size == factory.size()) {
               // The factory size will increase by one when we load a proper
               // plugin. If it didn't increase, then it's not a library we
               // are looking for.
               //cout << "Closing: " << lib_name << endl;
               dlclose(dlib);
               return NON_AVIA_LIBRARY;
          }

          dl_list_.insert(dl_list_.end(), dlib);
          return 0;
     }

     int finalize_libraries()
     {
          //cout << "------------------------------------------------" << endl;
          //cout << "Successfully loaded: " << endl;

          // create an array of the library names
          int i = 1;
          // factory is a global variable defined in AVIA main simulator
          for(fitr_=factory.begin(); fitr_!=factory.end(); fitr_++){
               lib_names_.insert(lib_names_.end(),
                                 fitr_->first);

               //cout << i << ": " << fitr_->first << endl;
               i++;
          }
          //cout << "----------------------------" << endl;
          return 0;
     }

     int open_library(std::string lib_name)
     {
          std::vector<std::string>::iterator it;
          it = std::find(lib_names_.begin(),lib_names_.end(),lib_name);
          if (it != lib_names_.end()) {
               // Calls the maker() function in appropriate cpp file in
               // dynamic library and puts new object into list.
               lib_list_.push_back(factory[*it]());
               return 0;
          }
          return -1;
     }

     T * object()
     {
          return lib_list_.front();
     }

     int close_libraries()
     {
          //// Causes hang probably because of destructors in loaded library
          //// and closing DDS participants abruptly
          ////// destroy any bridges we created
          //for(sitr_=lib_list_.begin();
          //    sitr_!=lib_list_.end();sitr_++){
          //     delete *sitr_;
          //}
          // close all the dynamic libs we opened
          for(itr_=dl_list_.begin(); itr_!=dl_list_.end(); itr_++){
               dlclose(*itr_);
          }
          return 0;
     }
protected:
private:
     std::list<void *> dl_list_;

     std::vector<string> lib_names_;
     typename std::map<string, M *, less<string> >::iterator fitr_;

     typename std::list<T*> lib_list_;
     typename std::list<T*>::iterator sitr_;

     list<void *>::iterator itr_;
};

#endif
