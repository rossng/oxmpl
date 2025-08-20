import { defineConfig } from 'vite';
import topLevelAwait from 'vite-plugin-top-level-await';
import wasm from 'vite-plugin-wasm';

export default defineConfig({
  plugins: [wasm(), topLevelAwait()],
  optimizeDeps: {
    exclude: ['oxmpl'],
  },
  build: {
    target: 'esnext',
  },
  test: {
    environment: 'node',
    testTimeout: 10000, // 10 seconds for motion planning tests
    hookTimeout: 10000,
  },
});
