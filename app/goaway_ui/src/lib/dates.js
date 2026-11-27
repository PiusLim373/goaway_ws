const pad = (n) => String(n).padStart(2, '0')

// YYMMDD_ seed for map name fields (tmms), so the operator only types the descriptive part.
// Computed per call: a browser left open overnight must not stamp yesterday.
export function todayPrefix(date = new Date()) {
  return `${pad(date.getFullYear() % 100)}${pad(date.getMonth() + 1)}${pad(date.getDate())}_`
}
