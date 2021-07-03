/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Author: Anatoly Baskeheev, Itseez Ltd, (myname.mysurname@mycompany.com)
 */

#ifndef _PCL_CUDA_TIMERS_HPP_
#define _PCL_CUDA_TIMERS_HPP_

#include <hip/hip_runtime_api.h>
#include <cstdio>

namespace pcl
{
    namespace gpu
    {
        struct Timer
        {
            hipEvent_t start_, stop_;
            Timer(bool runTimer = false) 
            { 
                hipEventCreate(&start_); 
                hipEventCreate(&stop_);  
                if (runTimer)
                    start();
            }
            ~Timer() 
            { 
                hipEventDestroy(start_);  
                hipEventDestroy(stop_);
            }

            void start() const { hipEventRecord(start_, 0); }
            Timer& stop()  { hipEventRecord(stop_, 0); hipEventSynchronize(stop_); return *this; }

            float time() const
            {
                float elapsed_time; 
                hipEventElapsedTime(&elapsed_time, start_, stop_);
                return elapsed_time;
            }
        };

        struct ScopeTimer
        {
            const char* name;
            hipEvent_t start, stop;
            ScopeTimer(const char* name_) : name(name_)
            {
                hipEventCreate(&start); 
                hipEventCreate(&stop);  
                hipEventRecord(start);
            }
            ~ScopeTimer()
            {
                float elapsed_time; 
                hipEventRecord(stop);	
                hipEventSynchronize(stop);
                hipEventElapsedTime(&elapsed_time, start, stop);
                printf("Time(%s) = %fms\n", name, elapsed_time);        
                hipEventDestroy(start);  
                hipEventDestroy(stop);
            }
        };
    }
}

#endif /* _PCL_CUDA_TIMERS_HPP_ */