import { defineConfig } from "vite";
import { copyFileSync, mkdirSync } from "node:fs";
import { resolve } from "node:path";

export default defineConfig({
  server: {
    host: "0.0.0.0",
    allowedHosts: ["terminal.local"],
  },
  plugins: [
    {
      name: "copy-classic-offline-scripts",
      closeBundle() {
        const target = resolve("dist/src");
        mkdirSync(target, { recursive: true });
        copyFileSync(resolve("src/pov-core.js"), resolve(target, "pov-core.js"));
        copyFileSync(resolve("src/app.js"), resolve(target, "app.js"));
      },
    },
  ],
});
