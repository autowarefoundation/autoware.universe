function contains(arr, o) {
    if (usages === null || usages === undefined) {
        return false;
    }
    const l = arr.length;
    for (let i = 0; i < l; i++) {
        if (arr[i] === o) {
            return true;
        }
    }
    return false;
}

function findEquation(el) {
    if (el.classList.contains('indep'))
        return null;
    while (el !== document) {
        if (el.classList.contains('equation')) {
            return el;
        }
        el = el.parentNode;
    }
    return null;
}

function isBranch(eq) {
    let el = eq.parentNode;
    while (el !== document && el.id !== 'algorithm') {
        if (el.classList.contains('condBody')) {
            return true;
        }
        el = el.parentNode;
    }
    return false;
}

function showEquations(eqId, level) {
    const el = document.getElementById('v' + eqId);
    if (el !== null) {
        const eq = findEquation(el);
        if (eq !== null && eq !== undefined) {
            eq.classList.remove('faded');
            if (!eq.classList.contains('depEq')) {
                eq.classList.add('depEq');
                if (level > 1)
                    eq.classList.add('faded2');
            } else {
                return; // been here already
            }
        }
    }

    const deps = var2dep[eqId];
    if (deps === undefined || deps === null) {
        return;
    }
    for (let i = 0; i < deps.length; i++) {
        const id = deps[i];
        showEquations(id, level + 1);
    }
}

function hideEquationForIds(ids, visibleId) {
    for (const co in ids) {
        if (visibleId === ids[co])
            continue;
        const el = document.getElementById(ids[co]);
        if (el !== null && el !== undefined) {
            const eq = findEquation(el);
            if (eq !== null && eq !== undefined)
                eq.classList.add('faded');
        }
    }
}
function clearAllClass(className) {
    const list = document.getElementsByClassName(className);
    if (list !== undefined) {
        while (list.length > 0) {
            list[0].classList.remove(className);
        }
    }
}

function clickHandler(e) {
    let t = e.target;

    clearAllClass('selectedProp');
    clearAllClass('faded');
    clearAllClass('faded2');
    clearAllClass('depEq');

    while (t !== document) {
        if (t.id !== null && t.id !== '' && t.id.charAt(0) === 'v') {
            const baseId = t.id.split('_')[0];
            const idval = baseId.substring(1);
            let el = document.getElementById(baseId);
            let n = 0;
            while (el !== null) {
                el.classList.add('selectedProp');
                n++;
                el = document.getElementById(baseId + '_' + n);
            }

            // fade other equations which do not use this variable
            usages = dep2var[idval];
            for (const i in var2dep) {
                if (i === idval)
                    continue;
                const vi = parseInt(i);
                if (!contains(usages, vi)) {
                    let el2 = document.getElementById('v' + i);
                    n = 0;
                    while (el2 !== null) {
                        const eq = findEquation(el2);
                        if (eq === null)
                            break;
                        eq.classList.add('faded');
                        if (!isBranch(eq))
                            break;
                        n++;
                        el2 = document.getElementById('v' + i + '_' + n);
                    }
                }
            }

            // unfade equations used to create that variable
            showEquations(idval, 0);

            hideEquationForIds(depConst, t.id);
            hideEquationForIds(depIsVar, t.id);
            break;
        }
        t = t.parentNode;
    }
}

document.addEventListener('DOMContentLoaded', function () {
    document.getElementById('algorithm').onclick = clickHandler;
}, false);