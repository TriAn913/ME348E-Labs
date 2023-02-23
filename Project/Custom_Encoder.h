#ifndef Custom_Encoder_h
#define Custom_Encoder_h

#include "Energia.h"

/// \brief Represents a single encoder.
///
/// An instance of this class represents a single encoder. The function begin
/// must be called before using any other function.
///
class Custom_Encoder
{
private:
	volatile int32_t enc_cnt = 0;
	volatile uint8_t enc_dir = 0;
	uint8_t _ea_pin;
	uint8_t _eb_pin;
	bool configured;
	void Trigger_Encoder();

public:
	Custom_Encoder();

	/// \brief Initialize the custom encoder class.
	///
	/// \param[in] ea_pin pin number on Launchpad connected to EA pin on encoder.
	/// \param[in] eb_pin pin number on Launchpad connected to EB pin on encoder.
	///
	/// This function needs to be called before any other function is used.
	void begin(uint8_t ea_pin, uint8_t eb_pin);

	/// \brief Return number of encoder ticks.
	///
	/// \return The number of encoder ticks where negative is backwards and positive is forwards.
	///
	int32_t getEncoderCnt();

	/// \brief Set the encoder tick count to 0
	///
	void resetEncoderCnt();
};
#endif