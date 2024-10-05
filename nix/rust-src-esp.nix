{ lib
, stdenv
, fetchurl
, zlib
}:
let
  libPath = lib.makeLibraryPath [ zlib stdenv.cc.cc.lib ];
in
stdenv.mkDerivation rec {
  pname = "rust-src-esp";
  version = "1.80.0.0";

  src = fetchurl {
    url = "https://github.com/esp-rs/rust-build/releases/download/v${version}/rust-src-${version}.tar.xz";
    hash = "sha256-7OIGrBpgOc8dJGTgdpu1AfV9J+VJ4N4Bw5X5l1psho8=";
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
