#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace robot::hw::lidar
{

class RingBuffer
{
private:
	std::vector<uint8_t> m_buffer;
	std::size_t m_capacity;
	std::size_t m_head;
	std::size_t m_tail;
	std::size_t m_size;
	bool m_has_overflowed;

	void discardOldestByte()
	{
		if (m_size == 0)
		{
			return;
		}

		m_head = (m_head + 1U) % m_capacity;
		--m_size;
		m_has_overflowed = true;
	}

protected:
public:
	explicit RingBuffer(std::size_t capacity = 0)
	: m_buffer(capacity, 0U),
	  m_capacity(capacity),
	  m_head(0U),
	  m_tail(0U),
	  m_size(0U),
	  m_has_overflowed(false)
	{
	}

	virtual ~RingBuffer() = default;

	void resize(std::size_t capacity)
	{
		m_buffer.assign(capacity, 0U);
		m_capacity = capacity;
		m_head = 0U;
		m_tail = 0U;
		m_size = 0U;
		m_has_overflowed = false;
	}

	std::size_t push(const uint8_t *data, std::size_t size)
	{
		if (data == nullptr || size == 0U || m_capacity == 0U)
		{
			return 0U;
		}

		for (std::size_t index = 0U; index < size; ++index)
		{
			if (m_size == m_capacity)
			{
				discardOldestByte();
			}

			m_buffer[m_tail] = data[index];
			m_tail = (m_tail + 1U) % m_capacity;
			++m_size;
		}

		return size;
	}

	bool peek(std::size_t index, uint8_t &value) const
	{
		if (index >= m_size || m_capacity == 0U)
		{
			return false;
		}

		const std::size_t actual_index = (m_head + index) % m_capacity;
		value = m_buffer[actual_index];
		return true;
	}

	std::size_t peek(uint8_t *destination, std::size_t count) const
	{
		if (destination == nullptr || count == 0U || m_capacity == 0U)
		{
			return 0U;
		}

		const std::size_t readable_count = count < m_size ? count : m_size;
		for (std::size_t index = 0U; index < readable_count; ++index)
		{
			destination[index] = m_buffer[(m_head + index) % m_capacity];
		}

		return readable_count;
	}

	std::size_t pop(uint8_t *destination, std::size_t count)
	{
		const std::size_t popped_count = peek(destination, count);
		consume(popped_count);
		return popped_count;
	}

	void consume(std::size_t count)
	{
		if (count == 0U || m_capacity == 0U || m_size == 0U)
		{
			return;
		}

		const std::size_t consumed_count = count < m_size ? count : m_size;
		m_head = (m_head + consumed_count) % m_capacity;
		m_size -= consumed_count;
	}

	std::size_t available() const
	{
		return m_size;
	}

	std::size_t capacity() const
	{
		return m_capacity;
	}

	void clear()
	{
		m_head = 0U;
		m_tail = 0U;
		m_size = 0U;
		m_has_overflowed = false;
	}

	bool overflowed() const
	{
		return m_has_overflowed;
	}

	void resetOverflowFlag()
	{
		m_has_overflowed = false;
	}
};

}  // namespace robot::hw::lidar
