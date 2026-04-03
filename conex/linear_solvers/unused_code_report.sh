#!/bin/bash
# Standalone unused code detection.
# Usage: ./unused_code_report.sh
set -uo pipefail
cd "$(dirname "$0")"

SRC_DIR=conex

echo "# Unused Code Report"
echo ""

# 1. Unused classes: declared in .h, never referenced in other files.
echo "## Dead classes"
echo "| Class | File | Notes |"
echo "|-------|------|-------|"
FOUND=false
while IFS= read -r match; do
  cls=$(echo "$match" | sed -E 's/.*class ([A-Z][A-Za-z_0-9]*).*/\1/')
  decl_file=$(echo "$match" | cut -d: -f1)
  decl_line=$(echo "$match" | cut -d: -f2)
  ref_count=$(grep -rl "\b${cls}\b" "$SRC_DIR" --include='*.h' --include='*.cc' 2>/dev/null \
    | grep -v "$decl_file" | wc -l)
  internal_uses=$(grep -n "\b${cls}\b" "$decl_file" 2>/dev/null \
    | grep -v "^${decl_line}:" \
    | grep -v '^\s*//' \
    | grep -v '^[0-9]*:\s*//' \
    | grep -v '#if 0' \
    | wc -l)
  if [ "$ref_count" -eq 0 ] && [ "$internal_uses" -le 0 ]; then
    echo "| \`${cls}\` | \`${decl_file}:${decl_line}\` | No references outside declaring file |"
    FOUND=true
  fi
done < <(grep -rn '^class [A-Z]' "$SRC_DIR" --include='*.h' | grep -v '//' | grep -v 'template')
$FOUND || echo "| (none found) | | |"

# 2. #if 0 blocks.
echo ""
echo "## Dead code blocks"
echo "| Location | Description |"
echo "|----------|-------------|"
FOUND=false
while IFS= read -r match; do
  file=$(echo "$match" | cut -d: -f1)
  line=$(echo "$match" | cut -d: -f2)
  desc=$(sed -n "$((line+1))p" "$file" | sed 's/^[[:space:]]*//' | head -c 60)
  [ -z "$desc" ] && desc="dead code block"
  echo "| \`${file}:${line}\` | ${desc} |"
  FOUND=true
done < <(grep -rn '#if 0' "$SRC_DIR" --include='*.h' --include='*.cc' 2>/dev/null)
# Commented-out using/template aliases.
while IFS= read -r match; do
  file=$(echo "$match" | cut -d: -f1)
  line=$(echo "$match" | cut -d: -f2)
  content=$(echo "$match" | cut -d: -f3- | sed 's/^[[:space:]]*//')
  echo "| \`${file}:${line}\` | \`${content}\` |"
  FOUND=true
done < <(grep -rn '^\s*//\s*\(using\|template\)' "$SRC_DIR" --include='*.h' --include='*.cc' 2>/dev/null)
$FOUND || echo "| (none found) | |"

# 3. Unused methods (hardcoded known suspects).
echo ""
echo "## Unused members/methods"
echo "| Item | File | Notes |"
echo "|------|------|-------|"
FOUND=false
for name in AddSupernode AddSeparator set_variable_indices variable_indices_; do
  decl=$(grep -rn "\b${name}\b" "$SRC_DIR" --include='*.h' | head -1)
  if [ -n "$decl" ]; then
    decl_file=$(echo "$decl" | cut -d: -f1)
    usage=$(grep -rn "\b${name}\b" "$SRC_DIR" --include='*.h' --include='*.cc' 2>/dev/null \
      | grep -v "$decl_file" | grep -v '^\s*//' | wc -l)
    if [ "$usage" -eq 0 ]; then
      line_num=$(echo "$decl" | cut -d: -f2)
      echo "| \`${name}\` | \`${decl_file}:${line_num}\` | Never called |"
      FOUND=true
    fi
  fi
done
$FOUND || echo "| (none found) | | |"

# 4. Dead free functions (defined but only referenced in defining file).
echo ""
echo "## Dead free functions"
echo "| Function | File | Notes |"
echo "|----------|------|-------|"
FOUND=false
for f in "$SRC_DIR"/common/*.cc "$SRC_DIR"/algorithms/*.cc "$SRC_DIR"/tree_solver/*.cc; do
  [ -f "$f" ] || continue
  while IFS= read -r fname; do
    [ -z "$fname" ] && continue
    total=$(grep -rn "\b${fname}\b" "$SRC_DIR" --include='*.cc' --include='*.h' 2>/dev/null | wc -l)
    if [ "$total" -eq 1 ]; then
      line=$(grep -n "\b${fname}\b" "$f" | head -1 | cut -d: -f1)
      echo "| \`${fname}\` | \`${f}:${line}\` | Defined but never called |"
      FOUND=true
    fi
  done < <(grep -oP '(?<=^)[A-Za-z_][A-Za-z_0-9]* (?=[A-Za-z_][A-Za-z_0-9]*\()' "$f" 2>/dev/null | sort -u)
done
$FOUND || echo "| (none found) | | |"

echo ""
echo "---"
echo "Generated: $(date -u '+%Y-%m-%d %H:%M UTC')"
