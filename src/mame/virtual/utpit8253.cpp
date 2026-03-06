// license:BSD-3-Clause
// copyright-holders:m1macrophage

#include "emu.h"
#include "machine/pit8253.h"

#define VERBOSE (LOG_GENERAL)
#define LOG_OUTPUT_FUNC osd_printf_info

#include "logmacro.h"

namespace {

class pittest_state : public driver_device
{
public:
	pittest_state(const machine_config &mconfig, device_type type, const char *tag) ATTR_COLD;

	void pittest(machine_config &config) ATTR_COLD;

	DECLARE_INPUT_CHANGED_MEMBER(key_pressed);

private:
	void print_sequence(const std::string &test_name) const;
	void verify(int mode, const std::string &test_name, const std::vector<int> &out, const std::vector<int> &counter);
	void verify(
		int mode, const std::string &test_name,
		const std::vector<int> &out,
		const std::unordered_set<int> &allowed_out,
		const std::vector<int> &counter);

	TIMER_CALLBACK_MEMBER(verify_deferred);
	TIMER_CALLBACK_MEMBER(print_final_report);

	TIMER_CALLBACK_MEMBER(sample_output_deferred);
	void clock(int state);
	void gate(int state);

	void test_mode_0();
	void test_mode_1();
	void test_mode_2();
	void test_mode_3();
	void test_mode_4();
	void test_mode_5();

	struct pit_state
	{
		const u16 counter = 0;
		const int output = 0;
		const int clock = 0;
	};

	struct test_data
	{
		const int mode;
		const std::string name;
		const std::vector<int> expected_output;
		const std::vector<int> expected_counter;
		const std::unordered_set<int> allowed_output;  // Output transitions allowed on these indices.
		bool passed = false;
	};

	required_device<pit8253_device> m_pit;
	std::vector<pit_state> m_states;
	std::vector<test_data> m_test_data;
	int m_last_pit_output;
};

pittest_state::pittest_state(const machine_config &mconfig, device_type type, const char *tag)
	: driver_device(mconfig, type, tag)
	, m_pit(*this, "pit")
	, m_states()
	, m_test_data()
	, m_last_pit_output(-1)
{
}

void pittest_state::pittest(machine_config &config)
{
	PIT8253(config, m_pit).out_handler<0>().set([this] (int state) { m_last_pit_output = state; });
}

void pittest_state::print_sequence(const std::string &test_name) const
{
	LOG("Test: %s\n", test_name.c_str());
	LOG("Output:  | ");
	for (const pit_state &s : m_states)
		if (!s.clock)
			LOG("%s | ", s.output ? "HI" : "LO");
	LOG("\n");

	LOG("Counter: | ");
	for (const pit_state &s : m_states)
		if (!s.clock)
			LOG("%02x | ", s.counter);
	LOG("\n");
}

void pittest_state::verify(int mode, const std::string &test_name, const std::vector<int> &out, const std::vector<int> &counter)
{
	verify(mode, test_name, out, {}, counter);
}

void pittest_state::verify(
	int mode, const std::string &test_name,
	const std::vector<int> &out, const std::unordered_set<int> &allowed_out,
	const std::vector<int> &counter)
{
	m_test_data.push_back(
	{
		.mode = mode,
		.name = test_name,
		.expected_output = out,
		.expected_counter = counter,
		.allowed_output = allowed_out,
	});
	machine().scheduler().synchronize(timer_expired_delegate(FUNC(pittest_state::verify_deferred), this), m_test_data.size() - 1);
}

TIMER_CALLBACK_MEMBER(pittest_state::verify_deferred)
{
	// 'true' validates that no counter or output transitions happen on positive CLK edges.
	constexpr bool CHECK_CLOCK_EDGE = true;

	print_sequence(m_test_data[param].name);
	const std::vector<int> &out = m_test_data[param].expected_output;
	const std::vector<int> &counter = m_test_data[param].expected_counter;
	const std::unordered_set<int> &allowed_out = m_test_data[param].allowed_output;
	if (out.size() != counter.size())
		fatalerror("pittest_state::verify - Incompatible `out` and `counter` sizes\n");

	int j = 0;
	bool passed = true;
	int last_out = -1;
	int last_counter = -1;
	std::string out_error;
	std::string counter_error;

	for (int i = 0; i < m_states.size() && passed; ++i)
	{
		if (m_states[i].clock)
		{
			if (CHECK_CLOCK_EDGE && last_out >= 0 && m_states[i].output != last_out && allowed_out.count(j) == 0)
			{
				passed = false;
				out_error = util::string_format(
					"Output changed on a positive clock edge. State index: %d, expected: %d, actual: %d",
					i, last_out, m_states[i].output);
			}
			if (CHECK_CLOCK_EDGE && last_counter >= 0 && m_states[i].counter != last_counter)
			{
				passed = false;
				counter_error = util::string_format(
					"Counter changed on a positive clock edge. State index: %d, expected: %d, actual: %d",
					i, last_counter, m_states[i].counter);
			}
		}
		else
		{
			if (j >= out.size())
				fatalerror("pittest_state::verify - `out` is too short.\n");
			if (out[j] >= 0 && out[j] != m_states[i].output)
			{
				passed = false;
				out_error = util::string_format(
					"Mismatched output at index: %d. Expected: %d, actual: %d",
					j, out[j], m_states[i].output);
			}
			if (counter[j] >= 0 && counter[j] != m_states[i].counter)
			{
				passed = false;
				counter_error = util::string_format(
					"Mismatched counter value at index: %d. Expected: %d, actual: %d",
					j, counter[j], m_states[i].counter);
			}
			last_out = m_states[i].output;
			last_counter = m_states[i].counter;
			++j;
		}
	}

	if (passed)
	{
		if (j != out.size())
			fatalerror("pittest_state::verify - `out` is too long.\n");
		LOG("Passed\n\n");
	}
	else
	{
		LOG("FAILED:\n");
		if (!out_error.empty())
			LOG("- %s\n", out_error.c_str());
		if (!counter_error.empty())
			LOG("- %s\n", counter_error.c_str());
		LOG("\n");
	}

	m_states.clear();
	m_test_data[param].passed = passed;
}

TIMER_CALLBACK_MEMBER(pittest_state::print_final_report)
{
	std::map<int, std::pair<int, int>> counts;  // mode -> {total test count, failure count}

	int n_tests = 0;
	for (const test_data &d : m_test_data)
	{
		if (!d.passed)
			++counts[d.mode].second;
		++counts[d.mode].first;
		++n_tests;
	}

	bool all_passed = true;
	for (const std::pair<const int, std::pair<int, int>> &c : counts)
	{
		if (c.second.second > 0)
		{
			LOG("Mode %d: %d/%d tests FAILED\n", c.first, c.second.second, c.second.first);
			all_passed = false;
		}
	}

	if (all_passed)
		LOG("All %d tests PASSED\n", n_tests);
}

TIMER_CALLBACK_MEMBER(pittest_state::sample_output_deferred)
{
	m_states.push_back(
	{
		.counter = m_pit->read(0x00),
		.output = m_last_pit_output,
		.clock = param
	});
}

void pittest_state::clock(int state)
{
	m_pit->write_clk0(state);
	machine().scheduler().synchronize(timer_expired_delegate(FUNC(pittest_state::sample_output_deferred), this), state);
}

void pittest_state::gate(int state)
{
	m_pit->write_gate0(state);
}

void pittest_state::test_mode_0()
{
	clock(0); gate(1);
	clock(1); clock(0);
	clock(1); m_pit->write(0x03, 0x10); clock(0);
	clock(1); clock(0);
	m_pit->write(0x00, 0x04);
	for (int i = 0; i < 7; ++i) { clock(1); clock(0); }
	verify(0, "Mode 0 - 8254 datasheet - 1",
		   { -1, -1,  0,  0, 0, 0, 0, 0, 1,    1,    1 },
		   { -1, -1, -1, -1, 4, 3, 2, 1, 0, 0xff, 0xfe });

	clock(0); gate(1);
	clock(1); clock(0);
	clock(1); m_pit->write(0x03, 0x10); clock(0);
	clock(1); clock(0);
	m_pit->write(0x00, 0x03);
	clock(1); clock(0);
	clock(1); gate(0); clock(0);
	clock(1); clock(0);
	clock(1); clock(0);
	gate(1);
	for (int i = 0; i < 3; ++i) { clock(1); clock(0); }
	verify(0, "Mode 0 - 8254 datasheet - 2",
		   { -1, -1,  0,  0, 0, 0, 0, 0, 0, 1,    1 },
		   { -1, -1, -1, -1, 3, 2, 2, 2, 1, 0, 0xff });

	clock(0); gate(1);
	clock(1); clock(0);
	clock(1); m_pit->write(0x03, 0x10); clock(0);
	clock(1); clock(0);
	m_pit->write(0x00, 0x03);
	for (int i = 0; i < 3; ++i) { clock(1); clock(0); }
	m_pit->write(0x00, 0x02);
	for (int i = 0; i < 4; ++i) { clock(1); clock(0); }
	verify(0, "Mode 0 - 8254 datasheet - 3",
	       { -1, -1,  0,  0, 0, 0, 0, 0, 0, 1,    1 },
	       { -1, -1, -1, -1, 3, 2, 1, 2, 1, 0, 0xff });

	clock(0); gate(1);
	m_pit->write(0x03, 0x10);
	for (int i = 0; i < 3; ++i) { clock(1); clock(0); }
	m_pit->write(0x00, 0x04);
	for (int i = 0; i < 10; ++i) { clock(1); clock(0); }
	verify(0, "Mode 0 - 8253 datasheet - 1",
		   { -1, -1,  0,  0, 0, 0, 0, 0, 1,    1,    1,    1,    1,    1 }, { 1 },
		   { -1, -1, -1, -1, 4, 3, 2, 1, 0, 0xff, 0xfe, 0xfd ,0xfc, 0xfb });

	clock(0); gate(1);
	m_pit->write(0x03, 0x10);
	for (int i = 0; i < 3; ++i) { clock(1); clock(0); }
	m_pit->write(0x00, 0x05);
	clock(1); clock(0);
	clock(1); clock(0); gate(0);
	clock(1); clock(0);
	clock(1); clock(0); gate(1);
	for (int i = 0; i < 6; ++i) { clock(1); clock(0); }
	verify(0, "Mode 0 - 8253 datasheet - 2",
	       { -1, -1,  0,  0, 0, 0,  0,  0, 0, 0, 0, 1,    1,    1 }, { 1 },  // Changed index 2 to 0, due to /wr timing not matching.
	       { -1, -1, -1, -1, 5, 4, -1, -1, 3, 2, 1, 0, 0xff, 0xfe });
}

void pittest_state::test_mode_1()
{
	clock(0); gate(0);
	clock(1); clock(0);
	clock(1); m_pit->write(0x03, 0x12); clock(0);
	clock(1); clock(0);
	m_pit->write(0x00, 0x03);
	clock(1); gate(1); clock(0); gate(0);
	for (int i = 0; i < 4; ++i) { clock(1); clock(0); }
	clock(1); gate(0); clock(0);
	gate(1); gate(0); clock(1); clock(0);
	clock(1); clock(0);
	verify(1, "Mode 1 - 8254 datasheet - 1",
		   { -1, -1,  1,  1,  1, 0, 0, 0, 1,    1, 0, 0 },
		   { -1, -1, -1, -1, -1, 3, 2, 1, 0, 0xff, 3, 2 });

	clock(0); gate(1);
	clock(1); clock(0);
	clock(1); m_pit->write(0x03, 0x12); clock(0);
	clock(1); clock(0);
	m_pit->write(0x00, 0x03);
	gate(0); clock(1); gate(1); clock(0); gate(0);
	clock(1); clock(0);
	clock(1); clock(0);
	clock(1); gate(1); clock(0);
	for (int i = 0; i < 4; ++i) { clock(1); clock(0); }
	verify(1, "Mode 1 - 8254 datasheet - 2",
		   { -1, -1,  1,  1,  1, 0, 0, 0, 0, 0, 0, 1 },
		   { -1, -1, -1, -1, -1, 3, 2, 1, 3, 2, 1, 0 });

	clock(0); gate(0);
	clock(1); clock(0);
	clock(1); m_pit->write(0x03, 0x12); clock(0);
	clock(1); clock(0);
	m_pit->write(0x00, 0x02);
	clock(1); gate(1); clock(0);
	clock(1); clock(0);
	clock(1); clock(0);
	m_pit->write(0x00, 0x04);
	clock(1); clock(0);
	clock(1); clock(0);
	gate(0); clock(1); clock(0);
	gate(1); clock(1), clock(0);
	clock(1); clock(0);
	verify(1, "Mode 1 - 8254 datasheet - 3",
		   { -1, -1,  1,  1,  1, 0, 0, 1,    1,    1, 0, 0 },
		   { -1, -1, -1, -1, -1, 2, 1, 0, 0xff, 0xfe, 4, 3 });

	clock(0); gate(0);
	m_pit->write(0x03, 0x12);
	for (int i = 0; i < 3; ++i) { clock(1); clock(0); }
	m_pit->write(0x00, 0x04);
	clock(1); clock(0);
	gate(1);
	clock(1); clock(0);
	for (int i = 0; i < 8; ++i) { clock(1); clock(0); }
	verify(1, "Mode 1 - 8253 datasheet - 1",
	       { -1, -1, -1, -1,  1, 0, 0, 0, 0, 1,    1,    1,    1,    1 }, { 1 },
	       { -1, -1, -1, -1, -1, 4, 3, 2, 1, 0, 0xff, 0xfe, 0xfd, 0xfc });

	clock(0); gate(0);
	m_pit->write(0x03, 0x12);
	clock(1); clock(0);
	clock(1); clock(0);
	gate(1); clock(1); clock(0);
	m_pit->write(0x00, 0x04);
	clock(1); clock(0); gate(0);
	clock(1); clock(0);
	gate(1); clock(1); clock(0);
	for (int i = 0; i < 7; ++i) { clock(1); clock(0); }
	verify(1, "Mode 1 - 8253 datasheet - 2",
		   { -1, -1,  1, 0, 0, 0, 0, 0, 0, 0, 1,    1,    1,    1 },
		   { -1, -1, -1, 4, 3, 2, 4, 3, 2, 1, 0, 0xff, 0xfe, 0xfd });

	clock(0); gate(0);
	m_pit->write(0x03, 0x12);
	clock(1); clock(0);  // osc
	m_pit->write(0x00, 0x01);
	clock(1); clock(0);  // osc
	gate(0); clock(1); // firmware
	gate(1); clock(0); // firmware
	clock(1);  // firmware
	clock(0);  // osc
	clock(1);  // firmware
	clock(0);  // osc
	clock(1);  // firmware
	clock(0);  // osc
	verify(1, "Mode 1 - sixtrak autotune 1",
		   { -1, -1, -1, 1, 0, 1,     1 },
		   { -1, -1, -1, -1, 1, 0, 0xff });

	clock(0); gate(1);
	m_pit->write(0x03, 0x12);
	clock(1); clock(0);  // osc
	m_pit->write(0x00, 0x03);
	clock(1); clock(0);  // osc
	clock(1); clock(0); clock(1); // osc
	gate(0); // firmware. Also sets clock(1), but that's already set above.
	gate(1); clock(0); // firmware
	clock(1);  // firmware
	clock(0);  // osc
	clock(1);  // firmware
	clock(0);  // osc
	clock(1);  // firmware
	clock(0);  // osc
	clock(1);  // firmware
	clock(0);  // osc
	verify(1, "Mode 1 - sixtrak autotune 2",
		   { -1, -1, -1, -1,  1, 0, 0, 0, 1 },
		   { -1, -1, -1, -1, -1, 3, 2, 1, 0 });

	clock(0); gate(0);
	m_pit->write(0x03, 0x12);
	m_pit->write(0x00, 0x08);
	clock(1); clock(0);
	clock(1); clock(0);
	gate(1); clock(1); clock(0);
	gate(0); clock(1); clock(0);
	clock(1); gate(1); clock(0);
	clock(1); clock(0);
	clock(1); clock(0);
	verify(1, "Mode 1 - gate / clock interaction",
		   { -1, -1,  1, 0, 0, 0, 0, 0 },
		   { -1, -1, -1, 8, 7, 6, 8, 7 });
}

void pittest_state::test_mode_2()
{
	clock(0); gate(1);
	clock(1); clock(0);
	clock(1); m_pit->write(0x03, 0x14); clock(0);
	clock(1); clock(0);
	m_pit->write(0x00, 0x03);
	for (int i = 0; i < 7; ++i) { clock(1); clock(0); }
	verify(2, "Mode 2 - 8254 datasheet - 1",
		   { -1, -1,  1,  1, 1, 1, 0, 1, 1, 0, 1 },
		   { -1, -1, -1, -1, 3, 2, 1, 3, 2, 1, 3 });

	clock(0); gate(1);
	clock(1); clock(0);
	clock(1); m_pit->write(0x03, 0x14); clock(0);
	clock(1); clock(0);
	m_pit->write(0x00, 0x03);
	clock(1); clock(0);
	clock(1); gate(0); clock(0);
	clock(1); clock(0);
	gate(1);
	for (int i = 0; i < 4; ++i) { clock(1); clock(0); }
	verify(2, "Mode 2 - 8254 datasheet - 2",
		   { -1, -1,  1,  1, 1, 1, 1, 1, 1, 0, 1 },
		   { -1, -1, -1, -1, 3, 2, 2, 3, 2, 1, 3 });

	clock(0); gate(1);
	clock(1); clock(0);
	clock(1); m_pit->write(0x03, 0x14); clock(0);
	clock(1); clock(0);
	m_pit->write(0x00, 0x04);
	for (int i = 0; i < 3; ++i) { clock(1); clock(0); }
	m_pit->write(0x00, 0x05);
	for (int i = 0; i < 4; ++i) { clock(1); clock(0); }
	verify(2, "Mode 2 - 8254 datasheet - 3",
		   { -1, -1,  1,  1, 1, 1, 1, 0, 1, 1, 1 },
		   { -1, -1, -1, -1, 4, 3, 2, 1, 5, 4, 3 });

	clock(0); gate(1);
	clock(1); clock(0);
	m_pit->write(0x03, 0x14);
	clock(1); clock(0);
	clock(1); clock(0);
	m_pit->write(0x00, 0x04);
	for (int i = 0; i < 7; ++i) { clock(1); clock(0); }
	clock(1); m_pit->write(0x00, 0x03); clock(0);
	for (int i = 0; i < 4; ++i) { clock(1); clock(0); }
	// next line, starting right after position 0(3).
	clock(1); gate(0); clock(0);
	for (int i = 0; i < 3; ++i) { clock(1); clock(0); }
	gate(1), clock(1); clock(0);
	for (int i = 0; i < 8; ++i) { clock(1); clock(0); }
	// The last output is "1" in the datasheet, that looks like a mistake. It should be "0".
	// The datasheet shows the points at which the counter resets as "0(4)", "0(3)", etc.
	// The value used here is 4, 3, etc.
	verify(2, "Mode 2 - 8253 datasheet",
		   { -1, -1,  1,  1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0 },
		   { -1, -1, -1, -1, 4, 3, 2, 1, 4, 3, 2, 1, 3, 2, 1, 3, 2, 2, 2, 2, 3, 2, 1, 3, 2, 1, 3, 2, 1 });
}

void pittest_state::test_mode_3()
{
	clock(0); gate(1);
	clock(1); clock(0);
	clock(1); m_pit->write(0x03, 0x16); clock(0);
	clock(1); clock(0);
	m_pit->write(0x00, 0x04);
	for (int i = 0; i < 10; ++i) { clock(1); clock(0); }
	verify(3, "Mode 3 - 8254 datasheet - 1",
		   { -1, -1,  1,  1, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1 },
		   { -1, -1, -1, -1, 4, 2, 4, 2, 4, 2, 4, 2, 4, 2 });

	clock(0); gate(1);
	clock(1); clock(0);
	clock(1); m_pit->write(0x03, 0x16); clock(0);
	clock(1); clock(0);
	m_pit->write(0x00, 0x05);
	for (int i = 0; i < 10; ++i) { clock(1); clock(0); }
	verify(3, "Mode 3 - 8254 datasheet - 2",
		   { -1, -1,  1,  1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 0 },
		   { -1, -1, -1, -1, 4, 2, 0, 4, 2, 4, 2, 0, 4, 2 });

	clock(0); gate(1);
	clock(1); clock(0);
	clock(1); m_pit->write(0x03, 0x16); clock(0);
	clock(1); clock(0);
	m_pit->write(0x00, 0x04);
	for (int i = 0; i < 4; ++i) { clock(1); clock(0); }
	gate(0); clock(1); clock(0);  // gate(0) will force the output high immediately, before cycle 8 ends.
	clock(1); clock(0); gate(1);
	for (int i = 0; i < 4; ++i) { clock(1); clock(0); }
	verify(3, "Mode 3 - 8254 datasheet - 3",
		   { -1, -1,  1,  1, 1, 1, 0, 0, 1, 1, 1, 1, 0, 0 }, { 8 },  // see comment above
		   { -1, -1, -1, -1, 4, 2, 4, 2, 2, 2, 4, 2, 4, 2 });

	// The waveforms in the 8253 datasheet are the same as the first and second
	// above. Except that that 8253 datasheet seems to have an error for "n = 5",
	// it shows the count going 5 - 4 - 2 - 5 - 2 - 5 ...
	// The correct one (8254 datasheet and MAME implementation) is: 4 - 2 - 0 - 4 - 2 - 4 ...
}

void pittest_state::test_mode_4()
{
	clock(0); gate(1);
	clock(1); clock(0);
	clock(1); m_pit->write(0x03, 0x18); clock(0);
	clock(1); clock(0);
	m_pit->write(0x00, 0x03);
	for (int i = 0; i < 7; ++i) { clock(1); clock(0); }
	verify(4, "Mode 4 - 8254 datasheet - 1",
		   { -1, -1,  1,  1, 1, 1, 1, 0,    1,    1,    1 },
		   { -1, -1, -1, -1, 3, 2, 1, 0, 0xff, 0xfe, 0xfd });

/*
	TODO: There is a known bug in the pit8253.cpp implementation: if the counter
	is written to while GATE is low, the timing of its initialization will not
	be correct. Enable test when that's fixed.

	clock(0); gate(0);
	clock(1); clock(0);
	clock(1); m_pit->write(0x03, 0x18); clock(0);
	clock(1); clock(0);
	m_pit->write(0x00, 0x03);
	clock(1); clock(0);
	clock(1); clock(0);
	clock(1); gate(1); clock(0);
	for (int i = 0; i < 4; ++i) { clock(1); clock(0); }
	verify(4, "Mode 4 - 8254 datasheet - 2",
		   { -1, -1,  1,  1, 1, 1, 1, 1, 1, 0,    1 },
		   { -1, -1, -1, -1, 3, 3, 3, 2, 1, 0, 0xff });
*/

	clock(0); gate(1);
	clock(1); clock(0);
	clock(1); m_pit->write(0x03, 0x18); clock(0);
	clock(1); clock(0);
	m_pit->write(0x00, 0x03);
	for (int i = 0; i < 3; ++i) { clock(1); clock(0); }
	m_pit->write(0x00, 0x02);
	for (int i = 0; i < 4; ++i) { clock(1); clock(0); }
	verify(4, "Mode 4 - 8254 datasheet - 3",
		   { -1, -1,  1,  1, 1, 1, 1, 1, 1, 0,    1 },
		   { -1, -1, -1, -1, 3, 2, 1, 2, 1, 0, 0xff });

	clock(0); gate(1);
	clock(1); clock(0);
	m_pit->write(0x03, 0x18);
	clock(1); clock(0);
	clock(1); clock(0);
	m_pit->write(0x00, 0x04);
	for (int i = 0; i < 10; ++i) { clock(1); clock(0); }
	verify(4, "Mode 4 - 8253 datasheet - 1",
		   { -1, -1,  1,  1, 1, 1, 1, 1, 0,    1,    1,    1,    1,    1 },
		   { -1, -1, -1, -1, 4, 3, 2, 1, 0, 0xff, 0xfe, 0xfd, 0xfc, 0xfb });

	clock(0); gate(1);
	clock(1); clock(0);
	m_pit->write(0x03, 0x18);
	clock(1); clock(0);
	clock(1); clock(0);
	m_pit->write(0x00, 0x04);
	clock(1); clock(0);
	gate(0); clock(1); clock(0);
	clock(1); clock(0);

	// The timing diagram has gate(1) just before clock(1), but this isn't
	// compatible with the shown counter values. Moving gate(1) after clock(1).
	clock(1); gate(1); clock(0);
	for (int i = 0; i < 6; ++i) { clock(1); clock(0); }
	verify(4, "Mode 4 - 8253 datasheet - 2",
		   { -1, -1,  1,  1, 1, 1, 1, 1, 1, 1, 1, 0,    1,    1 },
		   { -1, -1, -1, -1, 4 ,4, 4, 4, 3, 2, 1, 0, 0xff, 0xfe });
}

void pittest_state::test_mode_5()
{
	clock(0); gate(0);
	clock(1); clock(0);
	clock(1); m_pit->write(0x03, 0x1a); clock(0);
	clock(1); clock(0);
	m_pit->write(0x00, 0x03);
	clock(1); clock(0);
	gate(1); gate(0);
	for (int i = 0; i < 4; ++i) { clock(1); clock(0); }
	clock(1); gate(1); clock(0); gate(0);
	clock(1); clock(0);
	verify(5, "Mode 5 - 8254 datasheet - 1",
		   { -1, -1,  1,  1,  1, 1, 1, 1, 0,    1, 1 },
		   { -1, -1, -1, -1, -1, 3, 2, 1, 0, 0xff, 3 });

	clock(0); gate(0);
	clock(1); clock(0);
	clock(1); m_pit->write(0x03, 0x1a); clock(0);
	clock(1); clock(0);
	m_pit->write(0x00, 0x03);
	clock(1); clock(0);
	clock(1); clock(0);
	gate(1); gate(0);
	clock(1); clock(0);
	clock(1); gate(1); clock(0); gate(0);
	for (int i = 0; i < 5; ++i) { clock(1); clock(0); }
	verify(5, "Mode 5 - 8254 datasheet - 2",
		   { -1, -1,  1,  1,  1,  1, 1, 1, 1, 1, 1, 0,    1 },
		   { -1, -1, -1, -1, -1, -1, 3, 2, 3, 2, 1, 0, 0xff });

	clock(0); gate(0);
	clock(1); clock(0);
	clock(1); m_pit->write(0x03, 0x1a); clock(0);
	clock(1); clock(0);
	m_pit->write(0x00, 0x03);
	clock(1); clock(0);
	gate(1), gate(0);
	clock(1); clock(0);
	clock(1); clock(0);
	m_pit->write(0x00, 0x05);
	for (int i = 0; i < 4; ++i) { clock(1); clock(0); }
	gate(1); gate(0);
	clock(1); clock(0);
	clock(1); clock(0);
	verify(5, "Mode 5 - 8254 datashseet - 3",
		   { -1, -1,  1,  1,  1, 1, 1, 1, 0,    1,    1, 1, 1 },
		   { -1, -1, -1, -1, -1, 3, 2, 1, 0, 0xff, 0xfe, 5, 4 });

	clock(0); gate(0);
	m_pit->write(0x03, 0x1a);
	m_pit->write(0x00, 0x04);
	clock(1); clock(0);
	clock(1); clock(0);

	// Diagram shows clock(1); gate(1) (but very close). This is not compatible
	// with the diagrams and description in the 8254 datasheet. Moving gate(1)
	// before clock(1).
	gate(1); clock(1); clock(0);
	for (int i = 0; i < 10; ++i) { clock(1); clock(0); }
	verify(5, "Mode 5 - 8253 datasheet - 1",
		   { -1, -1,  1, 1, 1, 1, 1, 0,  1,  1,  1,  1,  1,  1 },
		   { -1, -1, -1, 4, 3, 2, 1, 0, -1, -1, -1, -1, -1, -1 });

	clock(0); gate(0);
	m_pit->write(0x03, 0x1a);
	m_pit->write(0x00, 0x04);
	clock(1); clock(0);
	clock(1); clock(0);
	gate(1); clock(1); clock(0);  // See "clock(1); gate(1)" note for the test case above.
	clock(1); gate(0); clock(0);
	gate(1); clock(1); clock(0);
	for (int i = 0; i < 8; ++i) { clock(1); clock(0); }
	verify(5, "Mode 5 - 8253 datasheet - 2",
		   { -1, -1,  1, 1, 1, 1, 1, 1, 1, 0,  1,  1,  1,  1 },
		   { -1, -1, -1, 4, 3, 4, 3, 2, 1, 0, -1, -1, -1, -1 });
}

DECLARE_INPUT_CHANGED_MEMBER(pittest_state::key_pressed)
{
	if (newval == 0)
		return;

	m_test_data.clear();
	m_states.clear();

	if (param == 10)
	{
		test_mode_0();
		test_mode_1();
		test_mode_2();
		test_mode_3();
		test_mode_4();
		test_mode_5();
	}
	else
	{
		switch (param)
		{
			case 0: test_mode_0(); break;
			case 1: test_mode_1(); break;
			case 2: test_mode_2(); break;
			case 3: test_mode_3(); break;
			case 4: test_mode_4(); break;
			case 5: test_mode_5(); break;
			default: return;
		}
	}

	machine().scheduler().synchronize(timer_expired_delegate(FUNC(pittest_state::print_final_report), this), 0);
}

INPUT_PORTS_START(pittest)
	PORT_START("KEY0")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("MODE 0") PORT_CODE(KEYCODE_0)
		PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(pittest_state::key_pressed), 0)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("MODE 1") PORT_CODE(KEYCODE_1)
		PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(pittest_state::key_pressed), 1)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("MODE 2") PORT_CODE(KEYCODE_2)
		PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(pittest_state::key_pressed), 2)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("MODE 3") PORT_CODE(KEYCODE_3)
		PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(pittest_state::key_pressed), 3)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("MODE 4") PORT_CODE(KEYCODE_4)
		PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(pittest_state::key_pressed), 4)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("MODE 5") PORT_CODE(KEYCODE_5)
		PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(pittest_state::key_pressed), 5)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_OTHER) PORT_NAME("ALL") PORT_CODE(KEYCODE_A)
		PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(pittest_state::key_pressed), 10)
INPUT_PORTS_END

ROM_START(pittest)
ROM_END

} // anonymous namespace

SYST(2000, pittest, 0, 0, pittest, pittest, pittest_state, empty_init, "Me", "PIT Tester", MACHINE_NO_SOUND_HW)
