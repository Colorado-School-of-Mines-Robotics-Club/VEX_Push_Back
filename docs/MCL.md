# Monte Carlo Localization / Particle filter

## Resampling algorithms

### Multinomial resampling

Simply draw particles from your pool with a pdf of their weights. This is simple and the most obvious but can cause problems. Say you have N particles, all with equal weight (representing an equal likelyhood of being at any of the points). Performing multinomial resampling would discard some of the points while duplicating others. Now the distribution of particles represents a higher likelyhood of being at the duplicated points and little likelyhood of being at the discarded points.

### Stratified resampling

Normalize all of your particle weights so that they all sum to 1. Now divide this range of \[0, 1] into N equal sections, where N is the number of particles. For each subsection, randomly sample from a uniform distribution inside of the subsection. Use the particle that corresponds to that random number, repeat for each subsection.

This solves the problem in multinomial resampling of having particles needlessly discarded. If we have all particles with equal weight, they will all get chosen 100% of the time. If weights are different, the ones with more weight will be chosen more often than ones with little weight, as expected.

### Systematic resampling

Same as stratified resampling, but instead of using a uniform distribution per subsection, simply use it for the first section and use the same number for every subsection. This can be thought of as adding an "offset" to the uniformly split subsections, and sampling on the boundaries.

This is effectively the same as stratified resampling, but has the benefit of avoiding extra random number generation with little loss to effectiveness.
