# twizy-control-2021

Bachelor's thesis project in path planning and obstacle recognition for a customized autonomous Renault Twizy at Chalmers university of technology, spring 2021.

# Getting started

How to get a local copy of this project on your computer.

## Prerequisites

* [Ubuntu](https://ubuntu.com/) - Ubuntu 18 is the current operating system on the Twizy. If your are using Windows it is recomended to dual boot Ubuntu.
* [ROS](https://www.ros.org/) - We're currently using ROS 1 (ROS Melodic Morenia) which is the current ROS version on the Twizy.

## Installation

1. Clone the repo:
   ```sh
   git clone https://github.com/OssianEriksson/twizy-control-2021.git
   ```

# Usage

See [documentation](doc).

# Contributing

1. Clone the project on your local computer (see [Getting Started](#getting-started)), or update existing local files with `git pull`.
2. Create your [branch](#branches) (`git checkout -b dev-my-amazing-new-feature`).
3. Make some changes.
4. Test those changes inside the newly created branch.
5. Push your new branch to the remote repo (github) with `git push -u origin dev-my-amazing-new-feature`
6. Open and merge a pull request through the [Pull requiests](https://github.com/OssianEriksson/twizy-control-2021/pulls) tab

## Branches

The branches in this project are:

* **master** - The stable and only longrunning branch. Code should have been confiremed to work before it is merged into master.
* **dev-\<name\>**, where \<name\> is a name of a feature to be implemented or a bug to be fixed - Create a new dev- branch when you need to change something in the code. After testing the updated code, this branch can get merged into master. The original dev- branch can then be deleted.
* **playground-\<name\>**, where \<name\> is your username or a description - Code in a playground- branch should be completely separate from production code. Playground code should be placed in the [playground](/playground) directory. playground- branches can get merged into master and then deleted once development on the branch is finished.

When creating a branch, name it either dev-\<name\> or playground-\<name\> according to the above:
```sh
git checkout -b dev-my-amazing-new-feature
```

## Playground

The [playground](playground) is for trying out and as the name suggest playing with code.
Code in playground should be kept completely separate from production code.
This directory is associated with playground-<name> branches, see [Branches](#branches).

# License

Distributed under the Apache 2.0 License.
See [LICENSE](/LICENCE) for more information.

# Contributors

* Jonathan Almgren
* Arvid Enliden
* Ask Uv
* Ossian Eriksson
* Albin Ekelund Carlsson
* Emil Hölvold

# Aknowledgements

All of previous years students who contributet to the Twizy!
