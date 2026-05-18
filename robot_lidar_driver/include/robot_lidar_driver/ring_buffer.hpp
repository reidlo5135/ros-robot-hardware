#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace robot::hw::lidar
{

class RingBuffer
{
private:
	std::vector<uint8_t> buffer_;
	std::size_t capacity_;
	std::size_t head_;
	std::size_t tail_;
	std::size_t size_;
	bool has_overflowed_;

	void discardOldestByte()
	{
		if (size_ == 0)
		{
			return;
		}

		head_ = (head_ + 1U) % capacity_;
		--size_;
		has_overflowed_ = true;
	}

protected:
public:
	explicit RingBuffer(std::size_t capacity = 0)
	: buffer_(capacity, 0U),
	  capacity_(capacity),
	  head_(0U),
	  tail_(0U),
	  size_(0U),
	  has_overflowed_(false)
	{
	}

	virtual ~RingBuffer() = default;

	void resize(std::size_t capacity)
	{
		buffer_.assign(capacity, 0U);
		capacity_ = capacity;
		head_ = 0U;
		tail_ = 0U;
		size_ = 0U;
		has_overflowed_ = false;
	}

	std::size_t push(const uint8_t *data, std::size_t size)
	{
		if (data == nullptr || size == 0U || capacity_ == 0U)
		{
			return 0U;
		}

		for (std::size_t index = 0U; index < size; ++index)
		{
			if (size_ == capacity_)
			{
				discardOldestByte();
			}

			buffer_[tail_] = data[index];
			tail_ = (tail_ + 1U) % capacity_;
			++size_;
		}

		return size;
	}

	bool peek(std::size_t index, uint8_t &value) const
	{
		if (index >= size_ || capacity_ == 0U)
		{
			return false;
		}

		const std::size_t actual_index = (head_ + index) % capacity_;
		value = buffer_[actual_index];
		return true;
	}

	std::size_t peek(uint8_t *destination, std::size_t count) const
	{
		if (destination == nullptr || count == 0U || capacity_ == 0U)
		{
			return 0U;
		}

		const std::size_t readable_count = count < size_ ? count : size_;
		for (std::size_t index = 0U; index < readable_count; ++index)
		{
			destination[index] = buffer_[(head_ + index) % capacity_];
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
		if (count == 0U || capacity_ == 0U || size_ == 0U)
		{
			return;
		}

		const std::size_t consumed_count = count < size_ ? count : size_;
		head_ = (head_ + consumed_count) % capacity_;
		size_ -= consumed_count;
	}

	std::size_t available() const
	{
		return size_;
	}

	std::size_t capacity() const
	{
		return capacity_;
	}

	void clear()
	{
		head_ = 0U;
		tail_ = 0U;
		size_ = 0U;
		has_overflowed_ = false;
	}

	bool overflowed() const
	{
		return has_overflowed_;
	}

	void resetOverflowFlag()
	{
		has_overflowed_ = false;
	}
};

}  // namespace robot::hw::lidar
