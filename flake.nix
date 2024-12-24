{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixpkgs-unstable";
    flake-parts = {
      url = "github:hercules-ci/flake-parts";
      inputs.nixpkgs-lib.follows = "nixpkgs";
    };
    fenix = {
      url = "github:nix-community/fenix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    crane.url = "github:ipetkov/crane/v0.19.0";
    esp-dev = {
      url = "github:mirrexagon/nixpkgs-esp-dev";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    treefmt-nix = {
      url = "github:numtide/treefmt-nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs =
    inputs@{
      self,
      nixpkgs,
      flake-parts,
      fenix,
      esp-dev,
      crane,
      ...
    }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      imports = [
        inputs.treefmt-nix.flakeModule
      ];

      flake = {
        overlays.default = import ./nix/overlay.nix;
      };
      systems = [ "x86_64-linux" ];
      perSystem =
        {
          config,
          self',
          system,
          ...
        }:
        let
          overlays = [
            fenix.overlays.default
            esp-dev.overlays.default
            self.overlays.default
          ];

          pkgs = import nixpkgs { inherit system overlays; };

          rustToolchain =
            with fenix.packages.${system};
            combine [
              pkgs.rust-esp
              pkgs.rust-src-esp
            ];

          craneLib = crane.mkLib pkgs;
          craneToolchain = craneLib.overrideToolchain rustToolchain;
          src = craneLib.cleanCargoSource ./.;
          commonArgs = {
            inherit src;
            cargoVendorDir = craneLib.vendorMultipleCargoDeps {
              cargoLockList = [
                ./Cargo.lock

                # Unfortunately this approach requires IFD (import-from-derivation)
                # otherwise Nix will refuse to read the Cargo.lock from our toolchain
                # (unless we build with `--impure`).
                #
                # Another way around this is to manually copy the rustlib `Cargo.lock`
                # to the repo and import it with `./path/to/rustlib/Cargo.lock` which
                # will avoid IFD entirely but will require manually keeping the file
                # up to date!
                "${pkgs.rust-src-esp}/lib/rustlib/src/rust/Cargo.lock"
              ];
            };

            strictDeps = true;
            doCheck = false;
            dontCheck = true;
            dontPatchELF = true;

            cargoExtraArgs = "-Zbuild-std=core,alloc --target xtensa-esp32-none-elf";

            buildInputs = with pkgs; [
              openssl
              pkg-config
              esp-idf-esp32-with-clang
            ];

            # FIXME: this is a hack for cargo to find our linker
            CARGO_TARGET_XTENSA_ESP32_NONE_ELF_LINKER =
              let
                deps = pkgs.esp-idf-esp32-with-clang.propagatedBuildInputs;
                idfName = pkgs.esp-idf-esp32-with-clang.name;
                targetPkg = builtins.head (builtins.filter (p: p.name == "esp-clang-${idfName}") deps);
              in
              "${targetPkg}/bin/xtensa-esp32-elf-ld";
          };

          cargoArtifacts = craneToolchain.buildDepsOnly commonArgs;

          crate = craneToolchain.buildPackage (
            commonArgs
            // {
              inherit cargoArtifacts;
            }
          );
        in
        {
          devShells.default =
            with pkgs;
            mkShell {
              buildInputs = [
                openssl
                pkg-config
                esp-idf-esp32-with-clang

                rustToolchain

                cargo-generate
                cargo-espflash
              ];
            };

          treefmt = {
            projectRootFile = "Cargo.toml";
            programs = {
              nixfmt.enable = true;
              rustfmt.enable = true;
            };
          };

          packages.default = crate;

          apps.default = {
            type = "app";
            program = pkgs.lib.getExe (
              pkgs.writeShellApplication {
                name = "espflash-run";
                runtimeInputs = [
                  crate
                  pkgs.espflash
                ];
                text = ''
                  espflash flash --monitor "${crate}/bin/${crate.pname}"
                '';
              }
            );
          };
        };
    };
}
