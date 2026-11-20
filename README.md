# Compás kernel

The Linux 4.4.94+ kernel work behind [Compás](https://github.com/Starnished66/compas-firmware) on Ingenic X1600 players (HiBy R1, R3 Pro II, R3 II 2025, TempoTec V1).

- `kit/`: the kernel kit that builds the kernel from the Ingenic X1600 SDK, with open-source replacements for the vendor drivers. See `kit/README.md`.
- Everything else: what Compás ships on top: board configs, kernel and module patches, open-module pins and build records. `KERNELS.md` says which kernel each board ships and how it was built.

The firmware repository includes this one as `firmware/kernel`.

## SDK

Ingenic no longer serves the SDK the kit builds from. The [`sdk-x1600-v6.0-20240606`](https://github.com/Starnished66/compas-kernel/releases/tag/sdk-x1600-v6.0-20240606) release carries an unmodified copy (`ingenic-linux-kernel4.4.94-x1600-v6.0-20240606.tar.bz2`, SHA-256 `3e8c101b7c12667dcfed888e2eda63cfa833c6e20f22569e2b646c40f8523641`, matching Ingenic's published MD5).

## Credits

The kit is Jepl4r's hiby-custom-kernel (removed from GitHub), whose history this repository keeps, and builds on MatthewBriggs's [R1 kernel work](https://github.com/MatthewBriggs/hiby-r1-linux-kernel-compiling).

## License

GPL-2.0, see `LICENSE`.
