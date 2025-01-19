{
  description = "Dependencies for benchmarking Pinocchio";

  inputs = {
    nixpkgs.url =  "github:nixos/nixpkgs/nixos-24.11";
  };

  outputs = inputs@{ self, ... }:
    let
      system = "x86_64-linux";
      overlays = [ ];
      
      pkgs = import inputs.nixpkgs {
        inherit system overlays;
      };

      benchmark = pkgs.stdenv.mkDerivation {
        pname = "rigidbody-benchmark";
        version = "0.0.1";
        src = ./.;
        
        buildInputs = [
          pkgs.cmake
          pkgs.pinocchio
          pkgs.eigen
        ];
      };
    in
      {
        packages.${system}.default = benchmark;
        apps.${system} = {
          kinematics = {type = "app"; program="${benchmark}/bin/kinematics";};
          rnea = {type = "app"; program="${benchmark}/bin/rnea";};
        };
        devShells = {        
          default = pkgs.mkShell {
            packages = [
              benchmark
            ];
          };
        };
      };
  nixConfig = {
    bash-prompt-prefix = "\\[\\e[38;5;12m\\](nix)\\[\\e[38;5;220m\\] ";
    extra-substituters = [
      "https://nix-community.cachix.org"
    ];
    extra-trusted-public-keys = [
      "nix-community.cachix.org-1:mB9FSh9qf2dCimDSUo8Zy7bkq5CX+/rkCWyvRCYg3Fs="
    ];
  };
}
