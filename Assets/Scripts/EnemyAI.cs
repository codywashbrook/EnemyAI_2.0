using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;
using UnityEngine.UI;

[RequireComponent(typeof(NavMeshAgent))]
public class EnemyAI : MonoBehaviour
{
    NavMeshAgent agent;
    Transform player; //used for rotating the stateUI towards the player
    public LayerMask whatIsEnemy; //what layers should this AI attack
    public Image stateUI; //image to show what current state is this AI

    public float rotationSpeed = 10;    //how fast the enemy turns to look at the player
    private Quaternion lookAtEnemy;    //rotation angle to look at player
    public float walkSpeed = 3; // walk speed
    public float chaseSpeed = 5; // chase speed
    //Patroling
    private Vector3 walkPoint;
    bool walkPointSet;
    float walkPointRange = 10;

    //Attacking
    public float timeBetweenAttacks; //the lower the number the faster the attacks 
    bool alreadyAttacked;
    public GameObject projectile;     //projectile to shoot player with

    //States
    public float sightRange, chaseRange , attackRange;
    [SerializeField] private bool rangedEnemy; //toggle if u wanted ranged enemy
    private float initialAttackRange;
    public bool enemyInSightRange, enemyInAttackRange; //checking if player detected or in attack range
    private float prevDetectRange;
    [SerializeField] private GameObject target; //current aquired target
    [SerializeField] private float timeToSwitchTarget = 2;
    private float targetSwitchTimer; //used for switching targets after some time

    private Vector3 spawnPos; //the first spawn position (used when retreating)
    private bool isChasing;
    [SerializeField] private float investigateTimer; //how much time should the AI investigate
    private float currentInvestigateTime;
    private Vector3 lastKnownPosition; //used to investigate player last known position
    private bool isRetreating;

    private void Awake()
    {
        player = GameObject.Find("Player").transform;
        agent = GetComponent<NavMeshAgent>();
        prevDetectRange = sightRange;
        initialAttackRange = attackRange;
        spawnPos = transform.position;
    }

    private void Update()
    {
        stateUI.transform.LookAt(player); //current state image facing the player at all times

        if (target && (enemyInSightRange || enemyInAttackRange))
        {
            lookAtEnemy = Quaternion.LookRotation(target.transform.position - transform.position);     //rotation angle to look at player & his allies
            //making sure to only rotate in Y axis
            lookAtEnemy.x = 0;
            lookAtEnemy.z = 0;
        }    

        SightRangeHandling();
        //Check for sight and attack range
        enemyInSightRange = Physics.CheckSphere(transform.position, sightRange, whatIsEnemy);
        enemyInAttackRange = Physics.CheckSphere(transform.position, attackRange, whatIsEnemy);

        targetSwitchTimer += Time.deltaTime;
        if (enemyInSightRange && targetSwitchTimer > timeToSwitchTarget)
        {
            targetSwitchTimer = 0; //target acquired, can only find new target after timeToSwitchTarget seconds
            Collider[] hitColliders = Physics.OverlapSphere(transform.position, sightRange, whatIsEnemy); //all player & his allies colliders in range
            target = GetClosestEnemy(hitColliders).transform.root.gameObject; //getting the closest enemy target
        }

        if (!target) //if not target aquired, reset the timer, which means the AI can aquire new target
            targetSwitchTimer = timeToSwitchTarget;

        if (!enemyInSightRange && !enemyInAttackRange && isRetreating) Retreating(); //retreat state
        else if (!enemyInSightRange && !enemyInAttackRange && isChasing) Investigate(); //investigate state
        else if (enemyInSightRange && !enemyInAttackRange) ChasePlayer(); //chase player state
        else if (enemyInAttackRange && enemyInSightRange) AttackPlayer(); //attack player state
        else if (!enemyInSightRange && !enemyInAttackRange) Patroling(); //patrol state
    }

    Collider GetClosestEnemy(Collider[] enemies) //checking the closest enemy collider (for targeting)
    {
        Collider tMin = null;
        float minDist = Mathf.Infinity;
        Vector3 currentPos = transform.position;
        foreach (Collider t in enemies)
        {
            Vector3 colPos = t.transform.position;
            float dist = Vector3.Distance(colPos, currentPos);
            if (dist < minDist)
            {
                tMin = t;
                minDist = dist;
            }
        }
        return tMin;
    }

    private void SightRangeHandling()
    {
        if (enemyInSightRange)    //if player sighted normally , change to chase range to prevent patrol/chase/run bug
        {
            sightRange = chaseRange;
        }
        else //back on normal patrol sight range
        {
            sightRange = prevDetectRange;
        }
    }

    private void Retreating()
    {
        agent.speed = walkSpeed;
        stateUI.color = new Color32(159, 74, 127, 255); //purple
        agent.SetDestination(spawnPos); //go to spawn position

        Vector3 distanceToRetreatPoint = transform.position - spawnPos;

        //spawnpoint reached
        if (distanceToRetreatPoint.magnitude < 3f)
            isRetreating = false;
    }

    private void Investigate()
    {
        agent.speed = walkSpeed;
        stateUI.color = new Color32(255, 255, 0, 255); //yellow
        currentInvestigateTime += Time.deltaTime;
        agent.SetDestination(lastKnownPosition);
        if (currentInvestigateTime >= investigateTimer)
        {
            isChasing = false;
            isRetreating = true; // AI is able to retreat after investigation is done
            currentInvestigateTime = 0;
        }
    }

    private void Patroling()
    {
        stateUI.color = new Color32(125, 255, 125, 255); //green
        agent.speed = walkSpeed;    //setting enemy speed to walk speed
        if (!walkPointSet)
            SearchWalkPoint();

        if (walkPointSet)
            agent.SetDestination(walkPoint);

        Vector3 distanceToWalkPoint = transform.position - walkPoint;

        //Walkpoint reached
        if (distanceToWalkPoint.magnitude < 3f)
            walkPointSet = false;
    }
    private void SearchWalkPoint()
    {
        //Calculate random point in range
        float randomZ = Random.Range(-walkPointRange, walkPointRange);
        float randomX = Random.Range(-walkPointRange, walkPointRange);

        walkPoint = new Vector3(transform.position.x + randomX, transform.position.y, transform.position.z + randomZ);

        if (SetDestination(walkPoint)) //checking if walkpoint is within the baked navmesh
        {
            walkPointSet = true;
        }
    }

    private bool SetDestination(Vector3 targetDestination)
    {
        NavMeshHit hit;
        if (NavMesh.SamplePosition(targetDestination, out hit, 1f, NavMesh.AllAreas))
        {
            agent.SetDestination(hit.position);
            return true;
        }
        return false;
    }

    private void ChasePlayer()
    {
        currentInvestigateTime = 0; //resetting this variable incase of detecting player while investigating
        isChasing = true; //used to check if AI was chasing before investigating
        isRetreating = false; //ressetting this bool incase of detecting player while retreating 
        stateUI.color = new Color32(255, 125, 11, 255); //orange

        attackRange = initialAttackRange;
        agent.speed = chaseSpeed;   //setting enemy speed to chase speed
        transform.rotation = Quaternion.Slerp(transform.rotation, lookAtEnemy, rotationSpeed * Time.deltaTime);
        if (target)
        {
            lastKnownPosition = target.transform.position; //this will be used in the investigation
            agent.SetDestination(target.transform.position); //chase target
        }         
    }

    private void AttackPlayer()
    {
        currentInvestigateTime = 0;
        isChasing = true;
        isRetreating = false;

        stateUI.color = new Color32(255, 0, 0, 255); //Red
        attackRange = initialAttackRange + 0.2f; // adding bit of an offset once target is within attack range (makes ai look better and less clunky)
        transform.rotation = Quaternion.Slerp(transform.rotation, lookAtEnemy, rotationSpeed * Time.deltaTime); //rotating towards the target

        if (target)
        {
            lastKnownPosition = target.transform.position;
            Vector3 distanceToTarget = transform.position - target.transform.position;

            //target reached
            if (distanceToTarget.magnitude < attackRange)
            {
                agent.SetDestination(transform.position);
            }
            else
            {
                agent.SetDestination(target.transform.position);
            }          
        }

        if (!alreadyAttacked)   //attack only if alreadyAttack is false (attack cooldown purpose)
        {
            if(rangedEnemy)
            {
                //ranged attack code
                Rigidbody rb = Instantiate(projectile, transform.position, Quaternion.identity).GetComponent<Rigidbody>();
                rb.AddForce(transform.forward * 32f, ForceMode.Impulse);
            }
            else
            {
                //melee attack code here
            }
            alreadyAttacked = true;
            Invoke(nameof(ResetAttack), timeBetweenAttacks);
        }
    }
    private void ResetAttack()
    {
        alreadyAttacked = false;    //which means enemy can attack
    }

    private void OnDrawGizmosSelected()     //check how attack and sight range looks
    {
        /*Gizmos.color = Color.red;
        Gizmos.DrawLine(transform.position, walkPoint);*/
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(transform.position, attackRange);
        Gizmos.color = Color.yellow;
        Gizmos.DrawWireSphere(transform.position, sightRange);
    }
}
