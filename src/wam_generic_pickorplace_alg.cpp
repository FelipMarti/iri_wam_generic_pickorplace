#include "wam_generic_pickorplace_alg.h"

WamGenericPickorplaceAlgorithm::WamGenericPickorplaceAlgorithm(void)
{
	pthread_mutex_init(&this->access_, NULL);
}

WamGenericPickorplaceAlgorithm::~WamGenericPickorplaceAlgorithm(void)
{
	pthread_mutex_destroy(&this->access_);
}

void
 WamGenericPickorplaceAlgorithm::config_update(Config & new_cfg, uint32_t level)
{
	this->lock();

	// save the current configuration
	this->config_ = new_cfg;

	this->unlock();
}

// WamGenericPickorplaceAlgorithm Public API
