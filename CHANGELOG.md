# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.2](https://github.com/acquire-project/acquire-driver-egrabber/compare/v0.1.1...v0.1.2) - 2023-06-27

### Added

- Nightly releases.

### Changes

- Updates call to `storage_properties_init` for the new function signature.
- Reflect a change in core-libs: `StorageProperties::chunking::max_bytes_per_chunk` is now a `uint64_t` (was
  a `uint32_t`).

## 0.1.1 - 2023-05-11
