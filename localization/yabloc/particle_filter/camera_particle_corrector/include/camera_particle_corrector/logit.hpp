#pragma once
namespace pcdless
{
float logit_to_prob(float logit);

/**
 * Convert probability to logit
 * This function is much faster than logit_to_prob() because it refers to pre-computeed table
 *
 * @param[in] prob
 * @return logit
 */
float prob_to_logit(float prob);
}  // namespace pcdless