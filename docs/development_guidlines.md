# Development Guidlines and Norms <!-- omit in toc -->

# Table of Contents <!-- omit in toc -->

- [Contributing](#contributing)
  - [Branches](#branches)
  - [Playground](#playground)
- [Suggested Reading](#suggested-reading)

# Contributing

1. Clone the project on your local computer (see [Getting Started](#getting-started)), or update existing local files with `git pull`.
2. Create your [branch](#branches) (`git checkout -b dev-my-amazing-new-feature`).
3. Make some changes.
4. Test those changes inside the newly created branch.
5. Push your new branch to the remote repo (github) with `git push -u origin dev-my-amazing-new-feature`
6. Open and merge a pull request from dev-my-amazing-new-featur to master through the [Pull requests](https://github.com/OssianEriksson/twizy-control-2021/pulls) tab

## Branches

The branches in this project are:

- **master** - The stable and only longrunning branch. Code should have been confiremed to work before it is merged into master.
- **dev-\<name\>**, where \<name\> is a name of a feature to be implemented or a bug to be fixed - Create a new dev- branch when you need to change something in the code. After testing the updated code, this branch can get merged into master. The original dev- branch can then be deleted.
- **playground-\<name\>**, where \<name\> is your username or a description - Code in a playground- branch should be completely separate from production code. Playground code should be placed in the [playground](/playground) directory. playground- branches can get merged into master and then deleted once development on the branch is finished.

When creating a branch, name it either dev-\<name\> or playground-\<name\> according to the above:
```sh
git checkout -b dev-my-amazing-new-feature
```

## Playground

The [playground](playground) is for trying out and as the name suggest playing with code.
Code in playground should be kept completely separate from production code.
This directory is associated with playground-<name> branches, see [Branches](#branches).

# Suggested Reading

- [Installing Python scripts and modules](http://docs.ros.org/en/jade/api/catkin/html/howto/format2/installing_python.html) - conventions for executable python scripts in packages
- [ROS rep-8](https://www.ros.org/reps/rep-0008.html)