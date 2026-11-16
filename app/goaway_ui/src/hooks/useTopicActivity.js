import { useState, useEffect, useRef } from 'react'
import { subscribe } from '../services/rosbridge'

// Returns { active, lastMsg } — active is true if topic published within timeoutMs.
// throttleMs is passed to rosbridge (see subscribe); keep it below timeoutMs.
export function useTopicActivity(topicName, messageType, timeoutMs = 500, throttleMs = 0) {
  const [active, setActive]   = useState(false)
  const [lastMsg, setLastMsg] = useState(null)
  const lastTimeRef = useRef(0)

  useEffect(() => {
    lastTimeRef.current = 0
    setActive(false)
    setLastMsg(null)

    const unsub = subscribe(topicName, messageType, (msg) => {
      lastTimeRef.current = Date.now()
      setLastMsg(msg)
    }, throttleMs)

    const interval = setInterval(() => {
      const isActive = lastTimeRef.current > 0 &&
                       (Date.now() - lastTimeRef.current) < timeoutMs
      setActive((prev) => (prev !== isActive ? isActive : prev))
    }, 200)

    return () => {
      unsub()
      clearInterval(interval)
    }
  }, [topicName, messageType, timeoutMs, throttleMs])

  return { active, lastMsg }
}
