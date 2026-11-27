import { defineConfig } from "vite";
import * as path from "node:path";

export default defineConfig(({ command }) =>
  command === "build"
    ? {
        publicDir: false,
        build: {
          minify: false,
          sourcemap: true,
          target: "es2022",
          lib: {
            formats: ["es"],
            entry: {
              index: "src/index.ts",
              vehicle: "src/vehicle.ts",
              curves: "src/curves.ts",
            },
            fileName: (_format, entryName) => `${entryName}.js`,
          },
          rollupOptions: {
            external: (id) => !id.startsWith(".") && !path.isAbsolute(id),
          },
        },
      }
    : {
        root: "example/",
        base: "./",
      },
);
