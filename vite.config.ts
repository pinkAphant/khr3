import { defineConfig } from "vite";
import * as path from "node:path";

export default defineConfig({
  publicDir: false,
  build: {
    minify: false,
    sourcemap: true,
    target: "es2022",
    emptyOutDir: true,
    lib: {
      formats: ["es"],
      entry: {
        index: "src/index.ts",
        vehicle: "src/vehicle.tsx",
      },
      fileName: (_format, entryName) => `${entryName}.js`,
    },
    rollupOptions: {
      external: (id) => !id.startsWith(".") && !path.isAbsolute(id),
    },
  },
});
