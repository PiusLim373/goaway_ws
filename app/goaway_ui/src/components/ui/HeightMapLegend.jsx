// Colour key for height_map_node's markers in Lichtblick (colours as in markers.cpp).
// Dots: where each value sent to the policy came from. Tiles: the map's height.
export const LEGEND_W = 84

const DOTS = [
  { color: '#00FFFF', label: 'Seen', hint: 'Hit by the LiDAR in the last second' },
  { color: '#3373FF', label: 'Remembered', hint: 'Seen earlier, kept in the map' },
  { color: '#FF9900', label: 'Neighbour', hint: 'Never seen, copied from a nearby cell' },
  { color: '#FF0000', label: 'Feet', hint: 'Nothing nearby, filled with the floor under the feet' },
  { color: '#FF00FF', label: 'Clamped', hint: 'Outside the trained range, clipped to it' },
]

// Turbo, as the tiles are painted: -0.3 m to +0.3 m around the floor under the feet
const TILE_GRADIENT =
  'linear-gradient(to right, #23171B, #3887F9, #2FE5AD, #96FA50, #FFB827, #E34613, #900D00)'

const headStyle = { color: 'var(--text-dim)', letterSpacing: '0.06em' }
const noteStyle = { color: 'var(--text-dim)', marginTop: -3, marginBottom: 2 }

export function HeightMapLegend() {
  return (
    <div
      className="flex flex-col gap-1"
      style={{ width: LEGEND_W, fontFamily: 'var(--font-mono)', fontSize: 9, color: 'var(--text)' }}
    >
      <span style={headStyle} title="Dots: the values sent to the policy">HEIGHT SCAN</span>
      <span style={noteStyle} title="Each value is base_link's height above the floor at that dot">
        from base_link
      </span>
      {DOTS.map(({ color, label, hint }) => (
        <span key={label} className="flex items-center gap-1.5" title={hint}>
          <span
            style={{
              width: 8,
              height: 8,
              borderRadius: '50%',
              background: color,
              boxShadow: '0 0 0 1px var(--border)',
              flexShrink: 0,
            }}
          />
          {label}
        </span>
      ))}

      <span style={{ ...headStyle, marginTop: 6 }} title="Tiles: map height around the floor under the feet">
        MAP HEIGHT
      </span>
      <span style={noteStyle} title="Tile height relative to the floor under the feet">from the feet</span>
      <span style={{ height: 6, borderRadius: 2, background: TILE_GRADIENT }} />
      <span className="flex justify-between" style={{ color: 'var(--text-dim)' }}>
        <span>-0.3 m</span>
        <span>+0.3 m</span>
      </span>
    </div>
  )
}
