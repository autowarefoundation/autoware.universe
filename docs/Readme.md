# Autoware (Architecture Proposal)

[![autoware](https://user-images.githubusercontent.com/8327598/69472442-cca50b00-0ded-11ea-9da0-9e2302aa1061.png)](https://youtu.be/kn2bIU_g0oY)

AutowareArchitectureProposal is a repository to explore and establish the architecture design of Autoware, an autonomous driving software managed by Autoware Foundation.

There already exists [Autoware.Auto](https://gitlab.com/autowarefoundation/autoware.auto) repository in Autoware Foundation GitLab. The architecture investigation, however, was done as a separate repository rather than a fork to explore architecture without prejudice from the existing source code.

The established architecture will be presented to Autoware.Auto which will then be reviewed by the community members and be used to improve Autoware.Auto. AutowareArchitectureProposal also contains new functions that do not yet exist in Autoware.Auto to verify that the architecture is feasible for various use cases. These functions are also planned to be refactored and merged into Autoware.Auto. The details are explained in the [Future plans](#future-plans) section.

## Note for non-Tier IV members

- All source code relating to this meta-repository is intended solely to demonstrate a potential new architecture for Autoware
- Please do not use any source code related to this repository to control actual vehicles
- While the documentation is published on GitHub Pages, the repository is private. We don't accept any contributions from outside of Tier IV

## Target

AutowareArchitectureProposal aims to realize autonomous driving in various environments.  
Autonomous driving on public roads is an extremely challenging project, and it cannot be achieved in a short period of time. Therefore we are trying to accumulate technology by applying the current AutowareArchitectureProposal to more restricted use cases such as in-factory transportation. At the same time, demonstration experiments in public roads are also in progress.

## Future plans

Again, autonomous driving is an extremely challenging project and this cannot be achieved in a short time. It cannot be achieved by only one company. People and companies all over the world need to work together to make it possible. We build the system and society through collaboration. So [Tier IV established the Autoware Foundation (AWF) in 2018](https://tier4.jp/cn/media/news/the-autoware-foundation/) to initiate, grow, and fund Autoware-related projects worldwide. [Autoware.Auto](https://www.autoware.auto/) is the current flagship AWF project and is a complete architectural redesign of [Autoware.AI](https://github.com/Autoware-AI/autoware.ai) that employs best-in-class software engineering practices.  
As part of Tier IV's commitment to collaboration with the AWF and its members, we plan to merge the additional functionality of AutowareArchitectureProposal to Autoware.Auto. Note that since Autoware.Auto has its own scope and ODD (Operational Design Domain, prerequisite environmental conditions for an automatic driving system to operate) that needs to be achieved, not all the features in AutowareArchitectureProposal will be required.

We keep using AutowareArchitectureProposal for some time, but remember that the core of our products will shift to Autoware.Auto.
