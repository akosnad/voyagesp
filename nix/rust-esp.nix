{
  lib,
  stdenv,
  fetchurl,
  zlib,
}:
let
  platform =
    {
      x86_64-linux = "x86_64-unknown-linux-gnu";
      aarch64-linux = "aarch64-unknown-linux-gnu";
      x86_64-darwin = "x86_64-apple-darwin";
      aarch64-darwin = "aarch64-apple-darwin";
    }
    .${stdenv.hostPlatform.system} or (throw "unsupported system ${stdenv.hostPlatform.system}");

  libPath = lib.makeLibraryPath [
    zlib
    stdenv.cc.cc.lib
  ];
in
stdenv.mkDerivation rec {
  pname = "rust-esp";
  version = "1.80.0.0";

  src = fetchurl {
    url = "https://github.com/esp-rs/rust-build/releases/download/v${version}/rust-${version}-${platform}.tar.xz";
    sha256 =
      {
        aarch64-linux = lib.fakeSha256;
        x86_64-linux = "sha256-08yHjWmTZFkFEyT3Ypjx/cVc4aRJLNkITM2vjVPPV9U=";
        x86_64-darwin = lib.fakeSha256;
        aarch64-darwin = lib.fakeSha256;
      }
      .${stdenv.hostPlatform.system};
  };

  installPhase = ''
    patchShebangs install.sh
    CFG_DISABLE_LDCONFIG=1 ./install.sh --prefix=$out

    rm $out/lib/rustlib/{components,install.log,manifest-*,rust-installer-version,uninstall.sh} || true

    ${lib.optionalString stdenv.isLinux ''
      if [ -d $out/bin ]; then
        for file in $(find $out/bin -type f); do
          if isELF "$file"; then
            patchelf \
              --set-interpreter ${stdenv.cc.bintools.dynamicLinker} \
              --add-rpath ${stdenv.cc.cc.lib}/lib \
              --add-rpath ${libPath} \
              "$file" || true
          fi
        done
      fi

      if [ -d $out/lib ]; then
        for file in $(find $out/lib -type f); do
          if isELF "$file"; then
            patchelf \
            --add-rpath ${stdenv.cc.cc.lib}/lib \
            --add-rpath ${libPath} \
            "$file" || true
          fi
        done
      fi

      if [ -d $out/libexec ]; then
        for file in $(find $out/libexec -type f); do
          if isELF "$file"; then
            patchelf \
              --set-interpreter ${stdenv.cc.bintools.dynamicLinker} \
              --add-rpath ${stdenv.cc.cc.lib}/lib \
              --add-rpath ${libPath} \
              "$file" || true
          fi
        done
      fi

      for file in $(find $out/lib/rustlib/*/bin -type f); do
        if isELF "$file"; then
          patchelf \
            --set-interpreter ${stdenv.cc.bintools.dynamicLinker} \
            --add-rpath ${stdenv.cc.cc.lib}/lib \
            --add-rpath ${libPath} \
            "$file" || true
        fi
      done
    ''}
  '';
  dontStrip = true;
}
