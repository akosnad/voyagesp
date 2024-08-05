{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    fenix = {
      url = "github:nix-community/fenix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { self, nixpkgs, flake-utils, fenix, ... }:
    flake-utils.lib.eachDefaultSystem
      (system:
        let
          pkgs = import nixpkgs {
            inherit system;
            overlays = [ fenix.overlays.default ];
          };

          fhs = pkgs.buildFHSUserEnv {
            name = "fhs-shell";
            targetPkgs = pkgs: with pkgs; [
              gcc

              pkg-config
              libclang.lib
              gnumake
              cmake
              ninja

              git
              wget

              rustup
              cargo-generate
              espup
              ldproxy

              espflash
              python3
              python3Packages.pip
              python3Packages.virtualenv

              rust-analyzer

              mosquitto
            ];
          };
        in
        {
          devShells.fhs = fhs.env;
          devShells.default = pkgs.mkShell {
            name = "default";
            buildInputs = [
              (pkgs.fenix.complete.withComponents [
                "cargo"
                "clippy"
                "rust-src"
                "rustc"
                "rustfmt"
              ])
              pkgs.rust-analyzer-nightly
            ];
          };
          formatter = pkgs.nixpkgs-fmt;
        }
      );
}
