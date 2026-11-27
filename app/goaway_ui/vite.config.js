import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import tailwindcss from '@tailwindcss/vite'

export default defineConfig({
  plugins: [react(), tailwindcss()],
  // Two entries, as tmms: navplan_tester is a standalone page with its own root (footer link
  // 🧭 Navplan), served at /navplan_tester by ui_backend's express.static `extensions`.
  build: {
    rollupOptions: {
      input: {
        main: 'index.html',
        navplan_tester: 'navplan_tester.html',
      },
    },
  },
  server: {
    host: '0.0.0.0',
    proxy: {
      '/api': {
        target: 'http://localhost:3001',
        changeOrigin: true,
      },
    },
  },
  optimizeDeps: {
    include: ['roslib'],
  },
})
