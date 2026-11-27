import { WS_URL } from './rosbridge'

// goaway_lichtblick (Caddy on :8080). ds/ds.url make Lichtblick auto-connect to rosbridge on
// load; layoutUrl re-fetches and re-applies the page's layout on every load, overriding
// whatever layout is cached in the browser (see goaway_lichtblick-compose.yaml).
const LICHTBLICK = `http://${window.location.hostname}:8080`

export const lichtblickUrl = (layout) => `${LICHTBLICK}/?ds=rosbridge-websocket`
  + `&ds.url=${encodeURIComponent(WS_URL)}`
  + `&layoutUrl=${encodeURIComponent(`${LICHTBLICK}/layouts/${layout}`)}`
