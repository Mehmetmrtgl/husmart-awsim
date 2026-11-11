// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Threading;
using ROS2;

namespace Awsim.Common
{
    /// <summary>
    /// A thread-safe timesource class that provides simulation time based on the dot net system utc time.
    /// This time source takes into account the value of the simulation timescale and
    /// starts at zero value when the simulation is started.
    ///
    /// Beytepe: sensor stamps are locked to Unity physics time.
    /// Unity runs several FixedUpdate (physics) steps back-to-back within one render frame, so a pure
    /// wall-clock stamp squeezes e.g. 30 ms of vehicle motion into ~1 ms of stamps. At 50 km/h that is a
    /// ~0.4 m mismatch between lidar/IMU/velocity stamps and the actual pose -> EKF/NDT disagree -> MRM.
    /// Fix: calls from the Unity main thread (all sensor publishers) return Time.timeAsDouble (== fixedTime
    /// inside FixedUpdate) plus an offset onto the wall-clock simulation time. Other threads (the /clock
    /// thread) still get the smooth wall-clock time, so /clock is unchanged. The offset tracks the smallest
    /// (clock - unity) seen, so stamps are never ahead of /clock (they trail it by up to about one frame,
    /// like real sensor latency); whenever it is re-synced the stamp equals the current clock, which is
    /// >= every earlier stamp, so stamps stay monotonic.
    /// </summary>
    public class DotNetSimulationTimeSource : ITimeSource
    {
        // If Unity time falls further than this behind the clock (e.g. a frame longer than
        // Time.maximumDeltaTime), the offset is re-synced.
        const double MaxLagSeconds = 0.2;

        DateTime _prevDateTime;
        double _time;
        bool _hasStarted = false;

        readonly Thread _mainThread;
        double _unityOffset;
        bool _hasUnityOffset = false;

        readonly object _lockObject = new object();

        public DotNetSimulationTimeSource()
        {
            _hasStarted = false;
            _mainThread = Thread.CurrentThread;   // created from AwsimRos2Node.Initialize() on the Unity main thread
        }

        public void GetTime(out int seconds, out uint nanoseconds)
        {
            lock (_lockObject)
            {
                DateTime currDateTime = DateTime.UtcNow;

                if (!_hasStarted)
                {
                    _hasStarted = true;
                    _time = 0.0;

                    _prevDateTime = currDateTime;
                }

                TimeSpan timeSpan = currDateTime - _prevDateTime;
                _prevDateTime = currDateTime;

                _time += timeSpan.TotalMilliseconds * 0.001f * ThreadSafeTime.TimeScale;

                double result = _time;
                if (Thread.CurrentThread == _mainThread)
                {
                    double unityTime = UnityEngine.Time.timeAsDouble;   // inside FixedUpdate this is the physics step time
                    double lag = _time - (unityTime + _unityOffset);
                    if (!_hasUnityOffset || lag < 0.0 || lag > MaxLagSeconds)
                    {
                        _unityOffset = _time - unityTime;   // result == clock now -> still monotonic
                        _hasUnityOffset = true;
                    }
                    result = unityTime + _unityOffset;
                }

                TimeUtility.TimeFromTotalSeconds(result, out seconds, out nanoseconds);
            }
        }
    }

}
