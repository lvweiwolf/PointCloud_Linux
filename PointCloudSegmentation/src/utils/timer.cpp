#include <src/utils/timer.h>

using namespace std::chrono;

namespace d3s {
	namespace pcs {

		Timer::Timer() : _started(false), _paused(false) {}

		void Timer::Start()
		{
			_started = true;
			_paused = false;
			_start_time = high_resolution_clock::now();
		}

		void Timer::Restart()
		{
			_started = false;
			Start();
		}

		void Timer::Pause()
		{
			_paused = true;
			_pause_time = high_resolution_clock::now();
		}

		void Timer::Resume()
		{
			_paused = false;
			_start_time += high_resolution_clock::now() - _pause_time;
		}

		void Timer::Reset()
		{
			_started = false;
			_paused = false;
		}

		double Timer::ElapsedMicroSeconds() const
		{
			if (!_started)
				return 0.0;

			if (_paused)
			{
				return duration_cast<microseconds>(_pause_time - _start_time).count();
			}
			else
			{
				return duration_cast<microseconds>(high_resolution_clock::now() - _start_time)
					.count();
			}
		}

		double Timer::ElapsedSeconds() const { return ElapsedMicroSeconds() / 1e6; }

		double Timer::ElapsedMinutes() const { return ElapsedSeconds() / 60; }


		double Timer::ElapsedHours() const { return ElapsedMinutes() / 60; }

		/*void Timer::PrintSeconds() const
		{
			std::cout << boost::format("耗时: %.5f [seconds]") % ElapsedSeconds() << std::endl;
		}

		void Timer::PrintMinutes() const
		{
			std::cout << boost::format("耗时: %.3f [minutes]") % ElapsedMinutes() << std::endl;
		}

		void Timer::PrintHours() const
		{
			std::cout << boost::format("耗时: %.3f [hours]") % ElapsedHours() << std::endl;
		}*/

	}
}