const TRANSITION_MESSAGES = {
  '1→2': { title: 'Robot will stand up', body: 'Ensure clear area around the robot. All personnel should stand back.' },
  '2→3': { title: 'Entering locomotion mode', body: 'Keep personnel clear of movement path. Robot will begin walking.' },
  '3→2': { title: 'Robot will stop locomotion', body: 'Robot will lock joints and stop walking.' },
  '2→1': { title: 'Robot will sit down', body: 'Ensure ground is clear below the robot.' },
  '2→4': { title: 'Robot will rise onto its rear legs', body: 'Keep personnel clear all around it. E-stop drops it from standing height.' },
  '4→2': { title: 'Robot will come down', body: 'Ensure the ground in front of the robot is clear: it lands on its front feet. An arm that is out comes back first.' },
}

export function WarningModal({
  open, fromMode, toMode, title, body, onConfirm, onCancel,
  heading = 'MODE CHANGE WARNING',
}) {
  if (!open) return null

  // Custom title/body (e.g. the locomotion switch) bypass the
  // fromMode/toMode transition lookup entirely.
  const isCustom = title != null || body != null
  const key = `${fromMode}→${toMode}`
  const msg = isCustom
    ? { title: title ?? 'Confirm', body: body ?? '' }
    : (TRANSITION_MESSAGES[key] ?? { title: 'Mode change', body: 'Keep the area clear.' })

  const MODE_LABELS = { 1: 'SIT', 2: 'STAND LOCK', 3: 'WALK', 4: 'UPRIGHT WALK' }

  return (
    <div
      className="fixed inset-0 z-50 flex items-center justify-center"
      style={{ background: 'rgba(0,0,0,0.75)', backdropFilter: 'blur(4px)' }}
      role="dialog"
      aria-modal="true"
    >
      <div
        className="rounded-lg p-6 w-80 flex flex-col gap-4"
        style={{
          background: 'var(--panel-bg)',
          border: '1px solid var(--accent)',
        }}
      >
        {/* Header */}
        <div className="flex items-center gap-2">
          <span style={{ color: '#EF4444', fontSize: 18 }}>⚠</span>
          <span
            style={{
              fontFamily: 'var(--font-mono)',
              fontSize: 11,
              letterSpacing: '0.08em',
              textTransform: 'uppercase',
              color: 'var(--text-dim)',
            }}
          >
            {heading}
          </span>
        </div>

        {/* Transition label */}
        {!isCustom && (
          <div
            style={{
              fontFamily: 'var(--font-mono)',
              fontSize: 13,
              color: 'var(--text-h)',
            }}
          >
            {MODE_LABELS[fromMode]} → {MODE_LABELS[toMode]}
          </div>
        )}

        {/* Message */}
        <div style={{ fontSize: 13, color: 'var(--text)' }}>
          <strong style={{ color: 'var(--text-h)' }}>{msg.title}.</strong>
          <br />
          {msg.body}
        </div>

        {/* Buttons */}
        <div className="flex gap-3 justify-end mt-1">
          <button
            className="btn-icon px-4 py-1.5 text-xs"
            onClick={onCancel}
            autoFocus
          >
            Cancel
          </button>
          <button
            className="px-4 py-1.5 rounded text-xs font-medium"
            style={{
              background: 'var(--accent-bright)',
              color: '#fff',
              fontFamily: 'var(--font-mono)',
              border: 'none',
              cursor: 'pointer',
            }}
            onClick={onConfirm}
          >
            Confirm ▶
          </button>
        </div>
      </div>
    </div>
  )
}
